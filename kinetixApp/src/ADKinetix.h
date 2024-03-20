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

//______________________________________________________________________________________________

#define KINETIX_CIRC_BUFF_SIZE 50

static const char *driverName = "ADKinetix";

//______________________________________________________________________________________________



//______________________________________________________________________________________________

#define KinetixTemperatureString             "KINETIX_TEMP"
#define KinetixFanSpeedString                "KINETIX_FAN_SPEED"

#define KinetixReadoutPortIdxString          "KINETIX_READOUT_PORT_IDX"
#define KinetixReadoutPortString          "KINETIX_READOUT_PORT"
#define KinetixSpeedIdxString             "KINETIX_SPEED_IDX"
#define KinetixSpeedString              "KINETIX_SPEED"
#define KinetixGainIdxString             "KINETIX_GAIN_IDX"
#define KinetixGainDescString             "KINETIX_GAIN_DESC"



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
    int KinetixReadoutPortIdx;
    int KinetixReadoutPort;
    int KinetixSpeedIdx;
    int KinetixSpeed;
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
    asynStatus selectSpeedTableMode(int speedTableIndex, int speedIndex, int gainIndex);

    void acquireStart();
    void acquireStop();
    void updateCameraRegion();


    CameraContext* cameraContext; 
    bool acquisitionActive = false;
    bool monitoringActive = false;
    int deviceIndex;
    void* frameBuffer;
    epicsThreadId monitorThreadId, acquisitionThreadId;
};

#define NUM_KINETIX_PARAMS ((int)(&LAST_KINETIX_PARAM - &FIRST_KINETIX_PARAM + 1))

//______________________________________________________________________________________________

#endif
