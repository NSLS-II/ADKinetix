#ifndef ADKINETIX_H
#define ADKINETIX_H

// version numbers
#define ADKINETIX_VERSION      0
#define ADKINETIX_REVISION     0
#define ADKINETIX_MODIFICATION 0


#include <stddef.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <mutex>
#include <condition_variable>

#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsEvent.h>
#include <epicsMutex.h>
#include <epicsString.h>
#include <epicsStdio.h>
#include <epicsMutex.h>
#include <cantProceed.h>
#include <epicsExit.h>
#include <iocsh.h>
#include <epicsExport.h>

#include "ADDriver.h"


/* PvCam includes */
#include "master.h"
#include "pvcam.h"

// PVCam examples helper data structures
#include "ADKinetixDS.h"

// Size of the circular buffer
#define KINETIX_CIRC_BUFF_SIZE 50

static const char *driverName = "ADKinetix";


/**
 * 
 */

// Temperature and Fan signals
#define KinetixTemperatureString             "KINETIX_TEMP"
#define KinetixFanSpeedString                "KINETIX_FAN_SPEED"


#define KinetixStopAcqOnTimeoutString       "KINETIX_STOP_ACQ_ON_TO"
#define KinetixWaitForFrameTimeoutString    "KINETIX_WAIT_FOR_FRAME_TO"
#define KinetixArmedString                  "KINETIX_ARMED"

#define KinetixApplyReadoutModeString       "KINETIX_APPLY_MODE"
#define KinetixModeValidString              "KINETIX_MODE_VALID"
#define KinetixReadoutPortIdxString          "KINETIX_READOUT_PORT_IDX"
#define KinetixReadoutPortDescString          "KINETIX_READOUT_PORT_DESC"
#define KinetixSpeedIdxString             "KINETIX_SPEED_IDX"
#define KinetixSpeedDescString              "KINETIX_SPEED_DESC"
#define KinetixGainIdxString             "KINETIX_GAIN_IDX"
#define KinetixGainDescString             "KINETIX_GAIN_DESC"


typedef enum {
    KINETIX_TRIG_INTERNAL = 0,
    KINETIX_TRIG_EDGE = 1,
    KINETIX_TRIG_GATE = 2,
} KINETIX_TRIGGER_MODE;



class ADKinetix : public ADDriver
{
public:

    ADKinetix(int deviceIndex, const char *portName, int maxSizeX, int maxSizeY, NDDataType_t dataType,
                int maxBuffers, size_t maxMemory, int priority, int stackSize);

    /* These are the methods that we override from ADDriver */
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    void report(FILE *fp, int details);
    void monitorThread();
    void acquisitionThread();
    ~ADKinetix ();
    bool waitForEofEvent(uns32 timeoutMs);


protected:
    int KinetixTemperature;
    #define FIRST_KINETIX_PARAM KinetixTemperature
    int KinetixFanSpeed;
    int KinetixStopAcqOnTimeout;
    int KinetixWaitForFrameTimeout;
    int KinetixArmed;

    int KinetixApplyReadoutMode;
    int KinetixModeValid;
    int KinetixReadoutPortIdx;
    int KinetixReadoutPortDesc;
    int KinetixSpeedIdx;
    int KinetixSpeedDesc;
    int KinetixGainIdx;
    int KinetixGainDesc;
    #define LAST_KINETIX_PARAM KinetixGainDesc 

    

private:
    void reportKinetixError(const char *functionName, const char *appMessage);

    bool isParamAvailable(int16 hcam, uns32 paramID, const char* paramName);
    bool readEnumeration(int16 hcam, NVPC* pNvpc, uns32 paramID, const char* paramName);
    void printSpeedTable();
    bool getSpeedTable();
    void updateImageFormat();
    void updateReadoutPortDesc();
    void selectSpeedTableMode();

    void acquireStart();
    void acquireStop();
    void updateCameraRegion();
    void getCurrentFrameDimensions(size_t* dims);

    NDDataType_t getCurrentNDBitDepth();
    CameraContext* cameraContext; 
    bool acquisitionActive = false;
    bool monitoringActive = false;
    int deviceIndex;
    void* frameBuffer = nullptr;
    epicsThreadId monitorThreadId, acquisitionThreadId;
};

#define NUM_KINETIX_PARAMS ((int)(&LAST_KINETIX_PARAM - &FIRST_KINETIX_PARAM + 1))

//______________________________________________________________________________________________

#endif