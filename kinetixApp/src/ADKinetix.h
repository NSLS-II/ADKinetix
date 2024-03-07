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

//______________________________________________________________________________________________

#define KINETIX_CIRC_BUFF_SIZE 50

static const char *driverName = "ADKinetix";

//______________________________________________________________________________________________

struct SpdtabGain
{
    
    int32 index;
    std::string name;
    int16 bitDepth;
};

struct SpdtabSpeed
{
    int32 index;
    uns16 pixTimeNs;
    std::vector<SpdtabGain> gains;
};

struct SpdtabPort
{
    int32 value;
    std::string name;
    std::vector<SpdtabSpeed> speeds;
};

struct Event
{
    std::mutex mutex;
    std::condition_variable cond;
    bool flag;
};

typedef struct EOFContext {
    int data1;

};

struct CameraContext
{
    char camName[CAM_NAME_LEN];
    bool isCamOpen;
    // All members below are initialized once the camera is successfully opened
    int16 hcam;
    uns16 sensorResX;
    uns16 sensorResY;
    rgn_type region;
    std::vector<SpdtabPort> speedTable;
    Event eofEvent;
    void* eofContext;
    FRAME_INFO eofFrameInfo;
    void* eofFrame;
};







// struct NVP
// {
//     int32 value;
//     std::string name;
// };
// typedef std::vector<NVP> NVPC;

// struct PvcamPpFeature
// {
//     int16 index;
//     uns32 id;
//     std::string name;
//     std::vector<PvcamPpParameter> parameterList;
// };

// struct PvcamPpParameter
// {
//     int16 index;
//     uns32 id;
//     std::string name;
//     uns32 minValue;
//     uns32 maxValue;
//     uns32 defValue;
// };

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
    void monitorThread();
    void acquisitionThread();
    ~ADKinetix ();
    bool waitForEofEvent(uns32 timeoutMs, bool &errorOccurred);

protected:
    int KinetixTemperature;
    #define FIRST_KINETIX_PARAM KinetixTemperature
    int KinetixFanSpeed;
    #define LAST_KINETIX_PARAM KinetixFanSpeed 

    

private:
    void reportKinetixError(const char *functionName, const char *appMessage);
    CameraContext* cameraContext; 
    bool alive = false;
    int deviceIndex;
    void* frameBuffer;
    epicsThreadId monitorThreadId, acquisitionThreadId;
};

#define NUM_KINETIX_PARAMS ((int)(&LAST_KINETIX_PARAM - &FIRST_KINETIX_PARAM + 1))

//______________________________________________________________________________________________

#endif
