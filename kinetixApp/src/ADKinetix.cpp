#include "ADKinetix.h"
/* PvCam includes */
#include "master.h"
#include "pvcam.h"

// Error message formatters
#define ERR(msg) asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s: %s\n", \
                           driverName, functionName, msg)

#define ERR_ARGS(fmt, ...) asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, \
                                     "%s::%s: " fmt "\n", driverName, functionName, __VA_ARGS__);

#define ERR_TO_STATUS(pvcamFunc) reportKinetixError(functionName, pvcamFunc);

// Flow message formatters
#define LOG(msg) asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s: %s\n", \
                           driverName, functionName, msg)

#define LOG_ARGS(fmt, ...) asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, \
                                     "%s::%s: " fmt "\n", driverName, functionName, __VA_ARGS__);



/**
 * @brief Function that instatiates a driver object. Called from IOC shell
 * 
 */
extern "C" int ADKinetixConfig(int deviceIndex, const char *portName, int maxSizeX, int maxSizeY, int dataType,
                               int maxBuffers, size_t maxMemory, int priority, int stackSize)
{
    new ADKinetix(deviceIndex, portName, maxSizeX, maxSizeY, (NDDataType_t)dataType, maxBuffers, maxMemory, priority, stackSize);
    return (asynSuccess);
}

//_____________________________________________________________________________________________

/**
 * @brief Wrapper C function passed to epicsThreadCreate to create temp monitor thread
 * 
 * @param drvPvt Pointer to instance of ADKinetix driver object
 */
static void monitorThreadC(void *drvPvt)
{
    ADKinetix *pPvt = (ADKinetix *)drvPvt;
    pPvt->monitorThread();
}

/**
 * @brief Wrapper C function passed to epicsThreadCreate to create acquisition thread
 * 
 * @param drvPvt Pointer to instance of ADKinetix driver object
 */
static void acquisitionThreadC(void *drvPvt)
{
    ADKinetix *pPvt = (ADKinetix *)drvPvt;
    pPvt->acquisitionThread();
}

/**
 * @brief Callback function called when exit is ran from IOC shell, deletes driver object instance
 * 
 * @param drvPvt Pointer to instance of ADKinetix driver object
 */
static void exitCallbackC(void *drvPvt)
{
    ADKinetix *pPvt = (ADKinetix *)drvPvt;
    delete pPvt;
}

//_____________________________________________________________________________________________
//_____________________________________________________________________________________________
// Public methods
//_____________________________________________________________________________________________
//_____________________________________________________________________________________________

/**
 * @brief Function that blocks until either timeout is reached, or new frame callback is detected
 * 
 * @param timeoutMs Time to wait before returning in ms
 * @return true if frame successfully was awaited
 * @return false if acquisition was aborted or timed out
 */
bool ADKinetix::waitForEofEvent(uns32 timeoutMs)
{
    std::unique_lock<std::mutex> lock(this->cameraContext->eofEvent.mutex);
    CameraContext *ctx = this->cameraContext; // For lambda we need temporary local reference to ctx pointer
    this->cameraContext->eofEvent.cond.wait_for(lock, std::chrono::milliseconds(timeoutMs),
                                                [ctx]()
                                                { return ctx->eofEvent.flag || ctx->threadAbortFlag; });
    if (!this->acquisitionActive)
    {
        printf("Processing aborted on camera %d\n", this->cameraContext->hcam);
        return false;
    }
    if (!this->cameraContext->eofEvent.flag)
    {
        printf("Camera %d timed out waiting for a frame\n", this->cameraContext->hcam);
        return false;
    }
    this->cameraContext->eofEvent.flag = false; // Reset flag
    if (!this->cameraContext->eofFrame)
    {
        return false;
    }
    return true;
}


/**
 * @brief Function that checks if camera supports SDK parameter
 * 
 * @param hcam device ID
 * @param paramID parameter ID
 * @param paramName parameter name
 * @return true if parameter is supported
 * @return false if parameter is not supported
 */
bool ADKinetix::isParamAvailable(int16 hcam, uns32 paramID, const char *paramName)
{
    if (!paramName)
    {
        return false;
    }

    rs_bool isAvailable;
    if (PV_OK != pl_get_param(hcam, paramID, ATTR_AVAIL, (void *)&isAvailable))
    {
        printf("Error reading ATTR_AVAIL of %s\n", paramName);
        return false;
    }
    if (isAvailable == FALSE)
    {
        printf("Parameter %s is not available\n", paramName);
        return false;
    }

    return true;
}

bool ADKinetix::readEnumeration(int16 hcam, NVPC *pNvpc, uns32 paramID, const char *paramName)
{
    const char *functionName = "readEnumeration";
    if (!pNvpc || !paramName)
        return false;

    if (!isParamAvailable(hcam, paramID, paramName))
        return false;

    uns32 count;
    if (PV_OK != pl_get_param(hcam, paramID, ATTR_COUNT, (void *)&count))
    {
        const std::string msg =
            "pl_get_param(" + std::string(paramName) + ")";
        ERR_TO_STATUS(msg.c_str());
        return false;
    }

    NVPC nvpc;
    for (uns32 i = 0; i < count; ++i)
    {
        // Retrieve the enum string length
        uns32 strLength;
        if (PV_OK != pl_enum_str_length(hcam, paramID, i, &strLength))
        {
            const std::string msg =
                "pl_enum_str_length(" + std::string(paramName) + ")";
            ERR_TO_STATUS(msg.c_str());
            return false;
        }

        // Allocate the destination string
        char *name = new (std::nothrow) char[strLength];
        if (!name)
        {
            printf("Unable to allocate memory for %s enum item name\n", paramName);
            return false;
        }

        // Get the string and value
        int32 value;
        if (PV_OK != pl_get_enum_param(hcam, paramID, i, &value, name, strLength))
        {
            const std::string msg =
                "pl_get_enum_param(" + std::string(paramName) + ")";
            ERR_TO_STATUS(msg.c_str());
            delete[] name;
            return false;
        }

        NVP nvp;
        nvp.value = value;
        nvp.name = name;
        nvpc.push_back(nvp);

        delete[] name;
    }
    pNvpc->swap(nvpc);

    return !pNvpc->empty();
}

void ADKinetix::selectSpeedTableMode()
{

    int readoutPortIdx, speedIdx, gainIdx;
    getIntegerParam(KinetixReadoutPortIdx, &readoutPortIdx);
    getIntegerParam(KinetixSpeedIdx, &speedIdx);
    getIntegerParam(KinetixGainIdx, &gainIdx);

    const char *functionName = "selectSpeedTableMode";
    updateReadoutPortDesc();

    if (PV_OK != pl_set_param(this->cameraContext->hcam, PARAM_READOUT_PORT,
                              (void *)&this->cameraContext->speedTable[readoutPortIdx].value))
    {
        ERR_TO_STATUS("Readout port could not be set");
        return;
    }
    LOG_ARGS("Readout port set to '%s'", this->cameraContext->speedTable[readoutPortIdx].name.c_str());

    if (PV_OK != pl_set_param(this->cameraContext->hcam, PARAM_SPDTAB_INDEX,
                              (void *)&this->cameraContext->speedTable[readoutPortIdx].speeds[speedIdx].index))
    {
        ERR_TO_STATUS("Readout speed index could not be set");
        return;
    }
    LOG_ARGS("Readout speed set to %d ns per pix", this->cameraContext->speedTable[readoutPortIdx].speeds[speedIdx].pixTimeNs);

    if (PV_OK != pl_set_param(this->cameraContext->hcam, PARAM_GAIN_INDEX,
                              (void *)&this->cameraContext->speedTable[readoutPortIdx].speeds[speedIdx].gains[gainIdx].index))
    {
        ERR_TO_STATUS("Gain index could not be set");
        return;
    }
    LOG_ARGS("Gain set to %s", this->cameraContext->speedTable[readoutPortIdx].speeds[speedIdx].gains[gainIdx].name.c_str());

    char readoutModeStr[256];
    snprintf(readoutModeStr, 256, "Mode: %s @ %d nspp w/ %d bpp", 
             this->cameraContext->speedTable[readoutPortIdx].name.c_str(), 
             this->cameraContext->speedTable[readoutPortIdx].speeds[speedIdx].pixTimeNs,
             this->cameraContext->speedTable[readoutPortIdx].speeds[speedIdx].gains[gainIdx].bitDepth);
    setStringParam(KinetixReadoutMode, readoutModeStr);

    if (this->cameraContext->speedTable[readoutPortIdx].speeds[speedIdx].gains[gainIdx].bitDepth != 8){
        this->cameraContext->imageFormat = PL_IMAGE_FORMAT_MONO16;
    } else this->cameraContext->imageFormat = PL_IMAGE_FORMAT_MONO8;

    callParamCallbacks();
}

void ADKinetix::printSpeedTable()
{
    const char *functionName = "printSpeedTable";
    if (this->cameraContext->speedTable.empty())
    {
        ERR("No speed table read from detector!");
        return;
    }
    // Speed table has been created, print it out
    printf("  Speed table:\n");
    for (const auto &port : this->cameraContext->speedTable)
    {
        printf("  - port '%s', value %d\n", port.name.c_str(), port.value);
        for (const auto &speed : port.speeds)
        {
            printf("    - speed index %d, running at %f MHz\n",
                   speed.index, 1000.0 / (float)speed.pixTimeNs);
            for (const auto &gain : speed.gains)
            {
                printf("      - gain index %d, %sbit-depth %d bpp\n",
                       gain.index,
                       (gain.name.empty()) ? "" : ("'" + gain.name + "', ").c_str(),
                       gain.bitDepth);
            }
        }
    }
    printf("\n");
}

bool ADKinetix::getSpeedTable()
{
    const char *functionName = "getSpeedTable";
    std::vector<SpdtabPort> table;

    NVPC ports;
    if (!readEnumeration(this->cameraContext->hcam, &ports, PARAM_READOUT_PORT, "PARAM_READOUT_PORT"))
        return false;

    if (!isParamAvailable(this->cameraContext->hcam, PARAM_SPDTAB_INDEX, "PARAM_SPDTAB_INDEX"))
        return false;
    if (!isParamAvailable(this->cameraContext->hcam, PARAM_PIX_TIME, "PARAM_PIX_TIME"))
        return false;
    if (!isParamAvailable(this->cameraContext->hcam, PARAM_GAIN_INDEX, "PARAM_GAIN_INDEX"))
        return false;
    if (!isParamAvailable(this->cameraContext->hcam, PARAM_BIT_DEPTH, "PARAM_BIT_DEPTH"))
        return false;

    rs_bool isGainNameAvailable;
    if (PV_OK != pl_get_param(this->cameraContext->hcam, PARAM_GAIN_NAME, ATTR_AVAIL,
                              (void *)&isGainNameAvailable))
    {
        printf("Error reading ATTR_AVAIL of PARAM_GAIN_NAME\n");
        return false;
    }
    const bool isGainNameSupported = isGainNameAvailable != FALSE;

    // Iterate through available ports and their speeds
    for (size_t pi = 0; pi < ports.size(); pi++)
    {
        // Set readout port
        if (PV_OK != pl_set_param(this->cameraContext->hcam, PARAM_READOUT_PORT,
                                  (void *)&ports[pi].value))
        {
            ERR_TO_STATUS("pl_set_param(PARAM_READOUT_PORT)");
            return false;
        }

        // Get number of available speeds for this port
        uns32 speedCount;
        if (PV_OK != pl_get_param(this->cameraContext->hcam, PARAM_SPDTAB_INDEX, ATTR_COUNT,
                                  (void *)&speedCount))
        {
            ERR_TO_STATUS("pl_get_param(PARAM_SPDTAB_INDEX)");
            return false;
        }

        SpdtabPort port;
        port.value = ports[pi].value;
        port.name = ports[pi].name;

        // Iterate through all the speeds
        for (int16 si = 0; si < (int16)speedCount; si++)
        {
            // Set camera to new speed index
            if (PV_OK != pl_set_param(this->cameraContext->hcam, PARAM_SPDTAB_INDEX, (void *)&si))
            {
                ERR_TO_STATUS("pl_set_param(PARAM_SPDTAB_INDEX)");
                return false;
            }

            // Get pixel time (readout time of one pixel in nanoseconds) for the
            // current port/speed pair. This can be used to calculate readout
            // frequency of the port/speed pair.
            uns16 pixTime;
            if (PV_OK != pl_get_param(this->cameraContext->hcam, PARAM_PIX_TIME, ATTR_CURRENT,
                                      (void *)&pixTime))
            {
                ERR_TO_STATUS("pl_get_param(PARAM_PIX_TIME)");
                return false;
            }

            uns32 gainCount;
            if (PV_OK != pl_get_param(this->cameraContext->hcam, PARAM_GAIN_INDEX, ATTR_COUNT,
                                      (void *)&gainCount))
            {
                ERR_TO_STATUS("pl_get_param(PARAM_GAIN_INDEX)");
                return false;
            }

            SpdtabSpeed speed;
            speed.index = si;
            speed.pixTimeNs = pixTime;

            // Iterate through all the gains, notice it starts at value 1!
            for (int16 gi = 1; gi <= (int16)gainCount; gi++)
            {
                // Set camera to new gain index
                if (PV_OK != pl_set_param(this->cameraContext->hcam, PARAM_GAIN_INDEX, (void *)&gi))
                {
                    ERR_TO_STATUS("pl_set_param(PARAM_GAIN_INDEX)");
                    return false;
                }

                // Get bit depth for the current gain
                int16 bitDepth;
                if (PV_OK != pl_get_param(this->cameraContext->hcam, PARAM_BIT_DEPTH, ATTR_CURRENT,
                                          (void *)&bitDepth))
                {
                    ERR_TO_STATUS("pl_get_param(PARAM_BIT_DEPTH)");
                    return false;
                }

                SpdtabGain gain;
                gain.index = gi;
                gain.bitDepth = bitDepth;

                if (isGainNameSupported)
                {
                    char gainName[MAX_GAIN_NAME_LEN];
                    if (PV_OK != pl_get_param(this->cameraContext->hcam, PARAM_GAIN_NAME,
                                              ATTR_CURRENT, (void *)gainName))
                    {
                        ERR_TO_STATUS("pl_get_param(PARAM_GAIN_NAME)");
                        return false;
                    }

                    gain.name = gainName;
                }

                speed.gains.push_back(gain);
            }

            port.speeds.push_back(speed);
        }

        table.push_back(port);
    }

    this->cameraContext->speedTable.swap(table);
    return true;
}

void ADKinetix::updateImageFormat()
{
    const char *functionName = "updateImageFormat";
    this->cameraContext->imageFormat = PL_IMAGE_FORMAT_MONO16;

    rs_bool isAvailable;
    if (PV_OK != pl_get_param(this->cameraContext->hcam, PARAM_IMAGE_FORMAT, ATTR_AVAIL,
                              (void *)&isAvailable))
        return;
    if (isAvailable == FALSE)
        return;

    int32 imageFormat;
    if (PV_OK != pl_get_param(this->cameraContext->hcam, PARAM_IMAGE_FORMAT, ATTR_CURRENT,
                              (void *)&imageFormat))
        return;

    this->cameraContext->imageFormat = imageFormat;
}

static void PV_DECL newFrameCallback(FRAME_INFO *pFrameInfo, CameraContext *ctx)
{

    if (ctx == nullptr)
        return;

    ctx->eofFrameInfo = *pFrameInfo;

    if (PV_OK != pl_exp_get_latest_frame(ctx->hcam, &ctx->eofFrame))
    {
        ctx->eofFrame = nullptr;
    }
    else
    {
        std::lock_guard<std::mutex> lock(ctx->eofEvent.mutex);
        ctx->eofEvent.flag = true;
    }
    ctx->eofEvent.cond.notify_all();
}

void ADKinetix::updateCameraRegion()
{
    const char *functionName = "updateCameraRegion";

    int regionStartX, regionStartY, regionStopX, regionStopY, binX, binY;

    getIntegerParam(ADMinX, &regionStartX);
    getIntegerParam(ADSizeX, &regionStopX);
    getIntegerParam(ADMinY, &regionStartY);
    getIntegerParam(ADSizeY, &regionStopY);
    getIntegerParam(ADBinX, &binX);
    getIntegerParam(ADBinY, &binY);

    this->cameraContext->region.s1 = (uns16)regionStartX;
    this->cameraContext->region.s2 = (uns16)regionStopX - 1;
    this->cameraContext->region.sbin = (uns16)binX;
    this->cameraContext->region.p1 = (uns16)regionStartY;
    this->cameraContext->region.p2 = (uns16)regionStopY - 1;
    this->cameraContext->region.pbin = (uns16)binY;

    LOG_ARGS("Configured Region: (%d, %d) to (%d, %d) | Binning: (%d, %d)",
             (int)this->cameraContext->region.s1,
             (int)this->cameraContext->region.p1,
             (int)this->cameraContext->region.s2,
             (int)this->cameraContext->region.p2,
             (int)this->cameraContext->region.sbin,
             (int)this->cameraContext->region.pbin)
}

void ADKinetix::getCurrentFrameDimensions(size_t *dims)
{
    dims[0] = (size_t)((this->cameraContext->region.s2 - this->cameraContext->region.s1 + 1) / this->cameraContext->region.sbin);
    dims[1] = (size_t)((this->cameraContext->region.p2 - this->cameraContext->region.p1 + 1) / this->cameraContext->region.pbin);
}

ADKinetix::ADKinetix(int deviceIndex, const char *portName, int maxSizeX, int maxSizeY, NDDataType_t dataType, int maxBuffers, size_t maxMemory, int priority, int stackSize)
    : ADDriver(portName, 1, NUM_KINETIX_PARAMS, maxBuffers, maxMemory, 0, 0, 0, 1, priority, stackSize)
{
    const char *functionName = "ADKinetix";
    asynStatus status = asynSuccess;

    createParam(KinetixTemperatureString,             asynParamFloat64,   &KinetixTemperature);
    createParam(KinetixFanSpeedString,                 asynParamInt32,   &KinetixFanSpeed);
    createParam(KinetixStopAcqOnTimeoutString, asynParamInt32, &KinetixStopAcqOnTimeout);
    createParam(KinetixWaitForFrameTimeoutString, asynParamInt32, &KinetixWaitForFrameTimeout);
    createParam(KinetixReadoutModeString, asynParamOctet, &KinetixReadoutMode);
    createParam(KinetixApplyReadoutModeString, asynParamInt32, &KinetixApplyReadoutMode);
    createParam(KinetixModeValidString, asynParamInt32, &KinetixModeValid);
    createParam(KinetixReadoutPortDescString,             asynParamOctet,   &KinetixReadoutPortDesc);
    createParam(KinetixReadoutPortIdxString,             asynParamInt32,   &KinetixReadoutPortIdx);
    createParam(KinetixSpeedDescString,             asynParamOctet,   &KinetixSpeedDesc);
    createParam(KinetixSpeedIdxString,             asynParamInt32,   &KinetixSpeedIdx);
    createParam(KinetixGainIdxString,             asynParamInt32,   &KinetixGainIdx);
    createParam(KinetixGainDescString,             asynParamOctet,   &KinetixGainDesc);

    if (!pl_pvcam_init())
    {
        reportKinetixError(functionName, "pl_pvcam_init");
        exit(1);
    }

    uns16 sdkVersion;
    int32 serialNumber;
    pl_pvcam_get_ver(&sdkVersion);
    char sdkVersionStr[40], fwVersionStr[40], modelStr[40], vendorStr[40], serialNumberStr[40];
    snprintf(sdkVersionStr, 40, "%d.%d.%d", (sdkVersion >> 8 & 0xFF), (sdkVersion >> 4 & 0xF), (sdkVersion >> 0 & 0xF));
    setStringParam(ADSDKVersion, sdkVersionStr);

    int16 numCameras = 0;
    if (!pl_cam_get_total(&numCameras))
    {
        ERR_TO_STATUS("pl_cam_get_total");
    }
    else if (numCameras <= 0)
    {
        ERR("No cameras detected!");
    }
    else if (numCameras < deviceIndex)
    {
        ERR_ARGS("There are only %d detected cameras! Cannot open camera with index %d!", numCameras, deviceIndex);
    }
    else
    {
        this->cameraContext = new (std::nothrow) CameraContext();
        if (!this->cameraContext)
        {
            ERR("Failed to create camera context!");
        }
        else
        {
            pl_cam_get_name(deviceIndex, this->cameraContext->camName);
            if (!pl_cam_open(this->cameraContext->camName, &this->cameraContext->hcam, OPEN_EXCLUSIVE))
            {
                ERR_ARGS("Failed to open camera with name %s", this->cameraContext->camName);
            }
            else
            {
                this->cameraContext->isCamOpen = true;
                LOG_ARGS("Opened camera...", this->cameraContext->camName);

                // Camera is opened, collect model information
                uns16 fwVersion;
                pl_get_param(this->cameraContext->hcam, PARAM_CAM_FW_VERSION, ATTR_CURRENT, (void *)&fwVersion);
                snprintf(fwVersionStr, 40, "%d.%d", (fwVersion >> 8) & 0xFF, (fwVersion >> 0) & 0xFF);
                setStringParam(ADFirmwareVersion, fwVersionStr);

                pl_get_param(this->cameraContext->hcam, PARAM_CAMERA_PART_NUMBER, ATTR_CURRENT, (void *)&serialNumber);
                snprintf(serialNumberStr, 40, "%d", serialNumber);
                setStringParam(ADSerialNumber, serialNumberStr);

                pl_get_param(this->cameraContext->hcam, PARAM_VENDOR_NAME, ATTR_CURRENT, (void *)vendorStr);
                setStringParam(ADManufacturer, vendorStr);

                pl_get_param(this->cameraContext->hcam, PARAM_PRODUCT_NAME, ATTR_CURRENT, (void *)modelStr);
                setStringParam(ADModel, modelStr);

                pl_get_param(this->cameraContext->hcam, PARAM_SER_SIZE, ATTR_CURRENT, (void *)&this->cameraContext->sensorResX);
                pl_get_param(this->cameraContext->hcam, PARAM_PAR_SIZE, ATTR_CURRENT, (void *)&this->cameraContext->sensorResY);

                setIntegerParam(ADMaxSizeX, this->cameraContext->sensorResX);
                setIntegerParam(ADMaxSizeY, this->cameraContext->sensorResX);
                LOG_ARGS("Model: %s | Resolution: %dx%d", modelStr, this->cameraContext->sensorResX, this->cameraContext->sensorResY);

                LOG("Configuring default region to full sensor size...");
                setIntegerParam(ADMinX, 0);
                setIntegerParam(ADMinY, 0);
                setIntegerParam(ADBinX, 1);
                setIntegerParam(ADBinY, 1);
                setIntegerParam(ADSizeX, this->cameraContext->sensorResX);
                setIntegerParam(ADSizeY, this->cameraContext->sensorResY);
                setIntegerParam(NDColorMode, NDColorModeMono);   // Only mono modes currently supported
                callParamCallbacks();
                updateCameraRegion();

                // Reset any pre-configured post-processing setup.
                pl_pp_reset(this->cameraContext->hcam);

                getSpeedTable();
                printSpeedTable();

                setIntegerParam(KinetixReadoutPortIdx, 0);
                setIntegerParam(KinetixSpeedIdx, 0);
                setIntegerParam(KinetixGainIdx, 0);
                callParamCallbacks();

                selectSpeedTableMode();

                if (PV_OK != pl_cam_register_callback_ex3(this->cameraContext->hcam, PL_CALLBACK_EOF, (void *)newFrameCallback, this->cameraContext))
                {
                    ERR("Failed to register callback function!");
                }
                else
                {
                    LOG("Registered EOF callback function.");
                    
                    this->monitoringActive = true;

                    LOG("Spawning camera monitor thread...");
                    epicsThreadOpts opts;
                    opts.priority = epicsThreadPriorityMedium;
                    opts.stackSize = epicsThreadGetStackSize(epicsThreadStackMedium);
                    opts.joinable = 1;

                    // Spawn the acquisition thread. Make sure it's joinable.
                    this->monitorThreadId = epicsThreadCreateOpt("acquisitionThread", (EPICSTHREADFUNC)monitorThreadC, this, &opts);
                }
            }
        }
    }

    epicsAtExit(exitCallbackC, (void *)this);
}

//_____________________________________________________________________________________________

//_____________________________________________________________________________________________

void ADKinetix::monitorThread()
{
    const char *functionName = "monitorThread";
    int acquiring;

    LOG("Temperature monitor thread active.");

    while (this->monitoringActive)
    {
        if (!this->acquisitionActive)
        {
            int16 temperature = 0;
            if (PV_OK != pl_get_param(this->cameraContext->hcam, PARAM_TEMP, ATTR_CURRENT, (void*) &temperature))
            {
                ERR_TO_STATUS("Failed to get temperature.");
            }
            else
            {
                setDoubleParam(KinetixTemperature, (double) (temperature / 100.0));
                callParamCallbacks();
            }
        }
        
        epicsThreadSleep(1);
    }
}

void ADKinetix::acquireStart()
{
    const char *functionName = "acquireStart";

    epicsThreadOpts opts;
    opts.priority = epicsThreadPriorityHigh;
    opts.stackSize = epicsThreadGetStackSize(epicsThreadStackBig);
    opts.joinable = 1;

    LOG("Spawning main acquisition thread...");
    // Spawn the acquisition thread. Make sure it's joinable.
    this->acquisitionThreadId = epicsThreadCreateOpt("acquisitionThread", (EPICSTHREADFUNC)acquisitionThreadC, this, &opts);
}

void ADKinetix::acquireStop()
{
    const char *functionName = "acquireStop";

    int acquisitionMode;
    if (this->acquisitionActive)
    {
        pl_exp_abort(this->cameraContext->hcam, CCS_HALT);
        this->acquisitionActive = false;
        setIntegerParam(ADAcquire, 0);
        LOG("Waiting for acquisition thread to join...");
        epicsThreadMustJoin(this->acquisitionThreadId);
        LOG("Acquisition stopped.");
        callParamCallbacks();
    }
}

NDDataType_t ADKinetix::getCurrentNDBitDepth()
{

    int readoutPortIdx, speedIdx, gainIdx;
    getIntegerParam(KinetixReadoutPortIdx, &readoutPortIdx);
    getIntegerParam(KinetixSpeedIdx, &speedIdx);
    getIntegerParam(KinetixGainIdx, &gainIdx);

    NDDataType_t dataType = NDUInt8;
    if (this->cameraContext->speedTable[readoutPortIdx].speeds[speedIdx].gains[gainIdx].bitDepth != 8)
    {
        dataType = NDUInt16;
    }
    return dataType;
}

void ADKinetix::updateReadoutPortDesc()
{
    int readoutPortIdx, speedIdx, gainIdx;
    getIntegerParam(KinetixReadoutPortIdx, &readoutPortIdx);
    getIntegerParam(KinetixSpeedIdx, &speedIdx);
    getIntegerParam(KinetixGainIdx, &gainIdx);

    int valid = 1;

    if (this->cameraContext->speedTable.size() > readoutPortIdx)
    { 
        setStringParam(KinetixReadoutPortDesc, this->cameraContext->speedTable[readoutPortIdx].name.c_str());
    }
    else
    {
        setStringParam(KinetixReadoutPortDesc, "Not Available");
        valid = 0;
    }
    if (valid == 1 && this->cameraContext->speedTable[readoutPortIdx].speeds.size() > speedIdx)
    {
        char speedDesc[40];
        snprintf(speedDesc, 40, "Pixel time: %d ns", this->cameraContext->speedTable[readoutPortIdx].speeds[speedIdx].pixTimeNs);
        setStringParam(KinetixSpeedDesc, speedDesc);
    }
    else
    {
        setStringParam(KinetixSpeedDesc, "Invalid");
        valid = 0;
    }
    if (valid == 1 && this->cameraContext->speedTable[readoutPortIdx].speeds[speedIdx].gains.size() > gainIdx)
    {
        char gainDescStr[40];
        snprintf(gainDescStr, 40, "%s, %d bpp", 
                 this->cameraContext->speedTable[readoutPortIdx].speeds[speedIdx].gains[gainIdx].name.c_str(),
                 this->cameraContext->speedTable[readoutPortIdx].speeds[speedIdx].gains[gainIdx].bitDepth);
        setStringParam(KinetixGainDesc, gainDescStr);
    }
    else
    {
        setStringParam(KinetixGainDesc, "Invalid");
        valid = 0;
    }

    if (valid == 1)
    {
        NDDataType_t dataType = getCurrentNDBitDepth();
        setIntegerParam(NDDataType, dataType);
        setIntegerParam(KinetixModeValid, 1);
    }
    else setIntegerParam(KinetixModeValid, 0);
    callParamCallbacks();
}

void ADKinetix::acquisitionThread()
{
    const char *functionName = "acquisitionThread";
    int acquiring, acquisitionMode, targetNumImages, collectedImages, 
        triggerMode, stopAcqOnTO, waitForFrameTO, modeValid;
    bool eofSuccess;
    double exposureTime, acquirePeriod;
    int16 pvcamExposureMode;
    NDArray *pArray;
    NDArrayInfo arrayInfo;
    size_t dims[2];
    NDColorMode_t colorMode = NDColorModeMono; // only grayscale at the moment
    NDDataType_t dataType = getCurrentNDBitDepth();
    getCurrentFrameDimensions(dims);

    getIntegerParam(KinetixModeValid, &modeValid);
    if (modeValid == 0)
    {
        ERR("Selected mode invalid! Check readout settings!");
        this->acquisitionActive = false;
        setIntegerParam(ADStatus, ADStatusError);
        setIntegerParam(ADAcquire, 0);
        callParamCallbacks();
        return;
    }

    getDoubleParam(ADAcquireTime, &exposureTime);
    getDoubleParam(ADAcquirePeriod, &acquirePeriod);
    double deadtime = 0;
    if(exposureTime < acquirePeriod) deadtime = acquirePeriod - exposureTime;

    getIntegerParam(ADTriggerMode, &triggerMode);
    getIntegerParam(ADImageMode, &acquisitionMode);
    getIntegerParam(ADNumImages, &targetNumImages);
    getIntegerParam(KinetixStopAcqOnTimeout, &stopAcqOnTO);
    getIntegerParam(KinetixWaitForFrameTimeout, &waitForFrameTO);

    // Convert EPICS selected trigger mode to PvCam format
    if (triggerMode == KINETIX_TRIG_INTERNAL)
        pvcamExposureMode = EXT_TRIG_INTERNAL | EXPOSE_OUT_FIRST_ROW;
    else if (triggerMode == KINETIX_TRIG_EDGE)
        pvcamExposureMode = EXT_TRIG_EDGE_RISING | EXPOSE_OUT_FIRST_ROW;
    else pvcamExposureMode = EXT_TRIG_LEVEL | EXPOSE_OUT_FIRST_ROW;

    uns32 frameBufferSize;
    const int16 circBuffMode = CIRC_OVERWRITE;

    this->acquisitionActive = true;
    LOG("Acquisition thread active.");

    LOG("Resetting image counter...");
    setIntegerParam(ADNumImagesCounter, 0);
    callParamCallbacks();

    if (acquisitionMode == ADImageSingle)
    {
        pl_exp_setup_seq(this->cameraContext->hcam, 1, 1, &this->cameraContext->region, pvcamExposureMode, (uns32)(exposureTime * 1000), &frameBufferSize);

        this->frameBuffer = (uns8 *)calloc(1, (size_t)frameBufferSize);
        LOG_ARGS("Allocated frame buffer of size %d...", frameBufferSize);

        if (PV_OK != pl_exp_start_seq(this->cameraContext->hcam, this->frameBuffer))
        {
            ERR_TO_STATUS("pl_exp_start_seq");
            this->acquisitionActive = false;
        }
    }
    else
    {
        pl_exp_setup_cont(this->cameraContext->hcam, 1, &this->cameraContext->region, pvcamExposureMode, (uns32)(exposureTime * 1000), &frameBufferSize, circBuffMode);

        this->frameBuffer = (uns8 *)calloc(KINETIX_CIRC_BUFF_SIZE, (size_t)frameBufferSize); // Allocate memory for circular buffer
        LOG_ARGS("Allocated circular buffer for %d frames. Total size: %d bytes.", KINETIX_CIRC_BUFF_SIZE, frameBufferSize * KINETIX_CIRC_BUFF_SIZE);

        if (PV_OK != pl_exp_start_cont(this->cameraContext->hcam, this->frameBuffer, KINETIX_CIRC_BUFF_SIZE * frameBufferSize))
        {
            ERR_TO_STATUS("pl_exp_start_cont");
            this->acquisitionActive = false;
        }
    }

    // In external trigger mode, set detector status to waiting before we get first frame
    if(triggerMode != KINETIX_TRIG_INTERNAL)
        setIntegerParam(ADStatus, ADStatusWaiting);
    else
        setIntegerParam(ADStatus, ADStatusAcquire);
    callParamCallbacks();


    while (acquisitionActive)
    {

        getIntegerParam(ADNumImagesCounter, &collectedImages);

        eofSuccess = this->waitForEofEvent((uns32) waitForFrameTO);
        if (eofSuccess)
        {
            // Allocate the NDArray
            this->pArrays[0] = pNDArrayPool->alloc(2, dims, dataType, 0, NULL);
            if (this->pArrays[0] != NULL)
            {
                pArray = this->pArrays[0];
            }
            else
            {
                this->pArrays[0]->release();
                ERR("Failed to allocate array!");
                setIntegerParam(ADStatus, ADStatusError);
                if(acquisitionMode == ADImageSingle)
                    pl_exp_finish_seq(this->cameraContext->hcam, this->frameBuffer, 0);
                else pl_exp_abort(this->cameraContext->hcam, CCS_HALT);
                callParamCallbacks();
                break;
            }

            collectedImages += 1;
            // New frame successfully collected.
            setIntegerParam(ADNumImagesCounter, collectedImages);
            updateTimeStamp(&pArray->epicsTS);

            pArray->getInfo(&arrayInfo);
            setIntegerParam(NDArraySize, (int)arrayInfo.totalBytes);
            setIntegerParam(NDArraySizeX, arrayInfo.xSize);
            setIntegerParam(NDArraySizeY, arrayInfo.ySize);

            setIntegerParam(ADStatus, ADStatusAcquire);

            // Copy data from eofFrame to pArray
            memcpy(pArray->pData, this->cameraContext->eofFrame, arrayInfo.totalBytes);

            // increment the array counter
            int arrayCounter;
            getIntegerParam(NDArrayCounter, &arrayCounter);
            arrayCounter++;
            setIntegerParam(NDArrayCounter, arrayCounter);

            // set the image unique ID to the number in the sequence
            pArray->uniqueId = arrayCounter;

            pArray->pAttributeList->add("ColorMode", "Color Mode", NDAttrInt32, &colorMode);

            // Sends image to the ArrayDataPV
            getAttributes(pArray->pAttributeList);

            // If we have a longer acq period than exposure, wait for difference
            if (deadtime > 0) LOG_ARGS("Expecting to sleep for %f", deadtime) //epicsThreadSleep(deadtime);

            doCallbacksGenericPointer(pArray, NDArrayData, 0);

            if (acquisitionMode == ADImageSingle)
            {
                pl_exp_finish_seq(this->cameraContext->hcam, this->frameBuffer, 0);
                this->acquisitionActive = false;
            }
            else if (acquisitionMode == ADImageMultiple && collectedImages == targetNumImages)
            {
                pl_exp_abort(this->cameraContext->hcam, CCS_HALT);
                this->acquisitionActive = false;
            }
            // Release the array
            pArray->release();
        }
        else if(stopAcqOnTO == 0)
        {
            // if we don't want to stop acq on a timeout, continue but set the status to waiting.
            LOG("Timeout waiting for next frame, trying again!");
            setIntegerParam(ADStatus, ADStatusWaiting);
        }
        else
        {
            // if we kill acq on TO, set status to aborting.
            setIntegerParam(ADStatus, ADStatusAborting);
            ERR("Failed to register EOF Event before timeout expired!");
            if(acquisitionMode == ADImageSingle)
                pl_exp_finish_seq(this->cameraContext->hcam, this->frameBuffer, 0);
            else pl_exp_abort(this->cameraContext->hcam, CCS_HALT);
            this->acquisitionActive = false;
        }
        // refresh all PVs
        callParamCallbacks();
    }

    LOG("Acquisition done. Freeing framebuffers.");
    free(this->frameBuffer);

    setIntegerParam(ADStatus, ADStatusIdle);
    setIntegerParam(ADAcquire, 0);
    callParamCallbacks();
}

//_____________________________________________________________________________________________

asynStatus ADKinetix::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    const char *functionName = "writeInt32";
    int function = pasynUser->reason;
    int detectorStatus;
    asynStatus status = asynSuccess;

    /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status = setIntegerParam(function, value);

    if (function == ADAcquire)
    {
        if (this->acquisitionActive && value == 0)
        {
            this->acquireStop();
        }
        else if (!this->acquisitionActive && value == 1)
        {
            this->acquireStart();
        }
        else if (value == 0)
        {
            ERR("Acquisition not active!");
            status = asynError;
        }
        else
        {
            ERR("Acquisition already active!");
            status = asynError;
        }
    }
    else if (function == KinetixGainIdx || function == KinetixSpeedIdx || function == KinetixReadoutPortIdx)
    {
        updateReadoutPortDesc();
    }
    else if (function == KinetixApplyReadoutMode)
    {
        LOG("Applying selected readout mode...");
        int readoutModeValid;
        getIntegerParam(KinetixModeValid, &readoutModeValid);
        if (readoutModeValid != 1)
        {
            ERR("Configured readout mode invalid!");
            status = asynError;
        }
        else
        {
            selectSpeedTableMode();
        }
    }
    else if (function == ADBinX || function == ADBinY || function == ADMinX
             || function == ADMinY || function == ADSizeX || function == ADSizeY)
    {
        // If we change binning or image dims, update the internal region state
        updateCameraRegion();
    }


    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();

    if (status != asynSuccess)
    {
        ERR_ARGS("error, status=%d function=%d, value=%f", status, function, value);
    }
    else
    {
        LOG_ARGS("function=%d, value=%f", function, value);
    }

    return status;
}

//_____________________________________________________________________________________________

asynStatus ADKinetix::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    const char *functionName = "writeFloat64";
    int function = pasynUser->reason;
    int status = asynSuccess;

    /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
     * status at the end, but that's OK */
    status = setDoubleParam(function, value);

    if (function == ADAcquireTime) {
        // if exposure time is set to something longer than acquire period, change acquire period to be the same
        double period;
        getDoubleParam(ADAcquirePeriod, &period);
        if (value > period){
            setDoubleParam(ADAcquirePeriod, value);
        }
    }

    /* Do callbacks so higher layers see any changes */
    callParamCallbacks();

    if (status)
    {
        ERR_ARGS("error, status=%d function=%d, value=%f", status, function, value);
    }
    else
    {
        LOG_ARGS("function=%d, value=%f", function, value);
    }

    return (asynStatus)status;
}

//_____________________________________________________________________________________________

void ADKinetix::report(FILE *fp, int details)
{

    const char *functionName = "report";

    fprintf(fp, "Kinetix %s\n", this->portName);
    if (details > 0)
    {
        int nx, ny, dataType;
        getIntegerParam(ADSizeX, &nx);
        getIntegerParam(ADSizeY, &ny);
        getIntegerParam(NDDataType, &dataType);
        fprintf(fp, "  NX, NY:            %d  %d\n", nx, ny);
        fprintf(fp, "  Data type:         %d\n", dataType);
    }

    /* Invoke the base class method */
    ADDriver::report(fp, details);
}

//_____________________________________________________________________________________________

ADKinetix::~ADKinetix()
{
    const char *functionName = "~ADKinetix";

    // Stop acquisiton if active
    this->acquireStop();

    // shut down separate threads
    this->monitoringActive = false;
    printf("Shutting down monitor thread...\n");
    epicsThreadMustJoin(this->monitorThreadId);

    // close camera if open
    if (this->cameraContext != NULL && this->cameraContext->isCamOpen)
    {
        pl_cam_close(this->cameraContext->hcam);
        printf("Closed camera...\n");
    }

    // delete camera context object
    delete this->cameraContext;

    // uninitialize SDK
    if (!pl_pvcam_uninit())
        reportKinetixError(functionName, "pl_pvcam_uninit");
    printf("Done.\n");
}

//_____________________________________________________________________________________________
//_____________________________________________________________________________________________
// Private methods
//_____________________________________________________________________________________________
//_____________________________________________________________________________________________

void ADKinetix::reportKinetixError(const char *functionName, const char *message)
{
    int16 code;
    char error[256];

    code = pl_error_code();
    pl_error_message(code, error);
    ERR_ARGS("Error Code %d | %s", code, error);
    setStringParam(ADStatusMessage, error);
    setIntegerParam(ADStatus, ADStatusError);
    callParamCallbacks();
}

//_____________________________________________________________________________________________

/* Code for iocsh registration */

/* pvCamConfig */
static const iocshArg ADKinetixConfigArg0 = {"DeviceIndex", iocshArgInt};
static const iocshArg ADKinetixConfigArg1 = {"Port name", iocshArgString};
static const iocshArg ADKinetixConfigArg2 = {"Max X size", iocshArgInt};
static const iocshArg ADKinetixConfigArg3 = {"Max Y size", iocshArgInt};
static const iocshArg ADKinetixConfigArg4 = {"Data type", iocshArgInt};
static const iocshArg ADKinetixConfigArg5 = {"maxBuffers", iocshArgInt};
static const iocshArg ADKinetixConfigArg6 = {"maxMemory", iocshArgInt};
static const iocshArg ADKinetixConfigArg7 = {"priority", iocshArgInt};
static const iocshArg ADKinetixConfigArg8 = {"stackSize", iocshArgInt};
static const iocshArg *const ADKinetixConfigArgs[] = {&ADKinetixConfigArg0,
                                                      &ADKinetixConfigArg1,
                                                      &ADKinetixConfigArg2,
                                                      &ADKinetixConfigArg3,
                                                      &ADKinetixConfigArg4,
                                                      &ADKinetixConfigArg5,
                                                      &ADKinetixConfigArg6,
                                                      &ADKinetixConfigArg7,
                                                      &ADKinetixConfigArg8};

static const iocshFuncDef configKinetix = {"ADKinetixConfig", 9, ADKinetixConfigArgs};

static void configKinetixCallFunc(const iocshArgBuf *args)
{
    ADKinetixConfig(args[0].ival, args[1].sval, args[2].ival, args[3].ival, args[4].ival,
                    args[5].ival, args[6].ival, args[7].ival, args[8].ival);
}

static void kinetixRegister(void)
{
    iocshRegister(&configKinetix, configKinetixCallFunc);
}

extern "C"
{
    epicsExportRegistrar(kinetixRegister);
}
