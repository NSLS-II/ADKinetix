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

//______________________________________________________________________________________________

static const char *driverName = "ADKinetix";

//______________________________________________________________________________________________

#define KinetixTemperatureString             "KINETIX_TEMP"
#define KinetixFanSpeedString                "KINETIX_FAN_SPEED"


class ADKinetix : public ADDriver
{
public:

    ADKinetix(int deviceIndex, const char *portName, int maxSizeX, int maxSizeY, NDDataType_t dataType,
                int maxBuffers, size_t maxMemory, int priority, int stackSize);

    /* These are the methods that we override from ADDriver */
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    void report(FILE *fp, int details);

    ~ADKinetix ();

protected:
    int KinetixTemperature;
    #define FIRST_KINETIX_PARAM KinetixTemperature
    int KinetixFanSpeed;
    #define LAST_KINETIX_PARAM KinetixFanSpeed 

    int deviceIndex;

private:
    void reportKinetixError (const char *functionName, const char *appMessage);
    void monitorThread();
    CameraContext* cameraContext; 
};

#define NUM_KINETIX_PARAMS ((int)(&LAST_KINETIX_PARAM - &FIRST_KINETIX_PARAM + 1))

//______________________________________________________________________________________________

#endif
