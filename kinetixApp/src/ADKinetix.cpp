#include "ADKinetix.h"



// Error message formatters
#define ERR(msg) asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "%s::%s: %s\n", \
    driverName, functionName, msg)

#define ERR_ARGS(fmt,...) asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, \
    "%s::%s: " fmt "\n", driverName, functionName, __VA_ARGS__);

// Flow message formatters
#define LOG(msg) asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, "%s::%s: %s\n", \
    driverName, functionName, msg)

#define LOG_ARGS(fmt,...) asynPrint(pasynUserSelf, ASYN_TRACE_FLOW, \
    "%s::%s: " fmt "\n", driverName, functionName, __VA_ARGS__);



//_____________________________________________________________________________________________

extern "C" int ADKinetixConfig(int deviceIndex, const char *portName, int maxSizeX, int maxSizeY, int dataType,
                                 int maxBuffers, size_t maxMemory, int priority, int stackSize)
{
    new ADKinetix(deviceIndex, portName, maxSizeX, maxSizeY, (NDDataType_t)dataType, maxBuffers, maxMemory, priority, stackSize);
    return(asynSuccess);
}

//_____________________________________________________________________________________________


static void monitorThreadC(void *drvPvt)
{
    ADKinetix *pPvt = (ADKinetix *)drvPvt;

    pPvt->montiorThread();
}

//_____________________________________________________________________________________________
//_____________________________________________________________________________________________
//Public methods
//_____________________________________________________________________________________________
//_____________________________________________________________________________________________

ADKinetix::ADKinetix(int deviceIndex, const char *portName, int maxSizeX, int maxSizeY, NDDataType_t dataType, int maxBuffers, size_t maxMemory, int priority, int stackSize)
    : ADDriver(portName, 1, NUM_KINETIX_PARAMS, maxBuffers, maxMemory, 0, 0, 0, 1, priority, stackSize)
{
    const char *functionName = "ADKinetix";
    int status = asynSuccess;

    createParam(KinetixTemperatureString,             asynParamFloat64,   &KinetixTemperature);
    createParam(KinetixFanSpeedString,                 asynParamInt32,   &KinetixFanSpeed);
    

    if (!pl_pvcam_init ())
    {
        reportKinetixError(functionName, "pl_pvcam_init");
        exit (1);
    }

    int16 numCameras = 0;
    if(!pl_cam_get_total(&numCameras)){
        reportKinetixError(functionName, "pl_cam_get_total");
    } else if(numCameras <=0){
        ERR("No cameras detected!");
    } else if(numCameras < deviceIndex) {
        ERR_ARGS("There are only %d detected cameras! Cannot open camera with index %d!", numCameras, deviceIndex);
    } else {
        this->cameraContext = new (std::nothrow) CameraContext(); 
        if(!this->cameraContext) {
            ERR("Failed to create camera context!");
        } else {
            pl_cam_get_name(deviceIndex, this->cameraContext->camName);
            if(!pl_cam_open(this->cameraContext->camName, &this->cameraContext->hcam, OPEN_EXCLUSIVE)){
                ERR_ARGS("Failed to open camera with name %s", this->cameraContext->camName);
            } else {
                this->cameraContext->isCamOpen = true;
                LOG_ARGS("Opened camera with name %s", this->cameraContext->camName);
                callParamCallbacks();
            }
        }
    }
}

//_____________________________________________________________________________________________



//_____________________________________________________________________________________________

void ADKinetix::monitorThread()
{

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
    
    return (asynStatus) status;
}

//_____________________________________________________________________________________________

void ADKinetix::report(FILE *fp, int details)
{

    const char* functionName = "report";

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

    if (this->cameraContext != NULL && this->cameraContext->isCamOpen){
        pl_cam_close(this->cameraContext->hcam);
    }
    
    delete this->cameraContext;

    if (!pl_pvcam_uninit ())
        reportKinetixError(functionName, "pl_pvcam_uninit");

}

//_____________________________________________________________________________________________
//_____________________________________________________________________________________________
//Private methods
//_____________________________________________________________________________________________
//_____________________________________________________________________________________________


void ADKinetix::reportKinetixError(const char *functionName, const char *message)
{
    int16 code;
    char error[256];

    code = pl_error_code();
    pl_error_message(code, error);
    ERR_ARGS("Error Code %d | %s", code, error);
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
static const iocshArg * const ADKinetixConfigArgs[] =  {&ADKinetixConfigArg0,
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

extern "C" {
    epicsExportRegistrar(kinetixRegister);
}

