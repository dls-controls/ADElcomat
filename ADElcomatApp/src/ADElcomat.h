#ifndef ADElcomat_H_
#define ADElcomat_H_

#include <epicsEvent.h>
#include "ADDriver.h"
#include "epicsThread.h"

#define FRAME_WIDTH 2

class ADElcomat : public ADDriver, public epicsThreadRunable
{
public:
    ADElcomat(const char* portName, const char* serialPortName,
        int serialPortAddress, int maxBuffers, size_t maxMemory, int priority,
        int stackSize);
    virtual ~ADElcomat();

    /* These are the methods that we override from asynPortDriver */
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);

    /* These are the methods that we override from epicsThreadRunable */
    virtual void run();

    /* These are the methods new to the class */
    bool connect();
    bool startStreaming();
    asynStatus stop();
    void flush();
    bool getDeviceInfo();
    bool parseDeviceInfo(const char *text, int *serialNumber, int *focalLength);
    bool processDataPoint();
    bool parseDataPoint(char *text, double *x, double *y, int *flags);
    bool dataCallback(double x, double y, int flags);
    void goDisconnected();
    void onDisconnected();
    void goIdle();
    void onIdle();
    void goAcquiring();
    void onAcquiring();

protected:
    char serialPortName[80];
    int serialPortAddress;
    epicsThread thread;
    asynUser *serialPortUser;
    epicsTime startTime;
    epicsEventId startEventId;
    epicsEventId stopEventId;
    void (ADElcomat::*state)();
    int errorsCount;

    /* Parameter indices */
    int index_connected;
    int index_serialNumber;
    int index_focalLength;
};

#endif /* ADElcomat_H_ */
