#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <epicsMutex.h>
#include <epicsString.h>
#include <epicsStdio.h>
#include <epicsMutex.h>
#include <epicsExport.h>
#include <iocsh.h>
#include "ADElcomat.h"
#include "asynOctetSyncIO.h"
#include "asynCommonSyncIO.h"

#define ADElcomat_connected_str      "CONNECTED"
#define ADElcomat_serialNumber_str   "SERIALNUMBER"
#define ADElcomat_focalLength_str    "FOCALLENGTH"

const double STOP_WAIT_TIMEOUT = 0.002;
const double DISCONNECT_DELAY = 5.0;

/* String terminators */
#define INPUT_EOS "\r"
#define OUTPUT_EOS "\n"

#define RX_BUFFER_SIZE 256
#define COMMAND_TIMEOUT 3.0

/* arc seconds to micro radians conversion factor */
#define ARCSECONDTOURAD 4.84813681

/*
 * Constructor
 */
ADElcomat::ADElcomat(const char* portName, const char* serialPortName,
        int serialPortAddress, int maxBuffers, size_t maxMemory, int priority,
        int stackSize)
    : ADDriver(portName, 1 /*maxAddr*/, 100, maxBuffers, maxMemory,
        asynInt32Mask | asynFloat64ArrayMask,
        asynInt32Mask | asynFloat64ArrayMask,
        0, /*ASYN_CANBLOCK=0, ASYN_MULTIDEVICE=0 */
        1, /*autoConnect*/
        priority, /*default priority */
        stackSize) /*default stack size*/
    , thread(*this, "rxthread", epicsThreadGetStackSize(epicsThreadStackMedium))
    , errorsCount(0)
{
    /* create the parameters */
    createParam(ADElcomat_connected_str, asynParamInt32, &index_connected);
    createParam(ADElcomat_serialNumber_str, asynParamInt32, &index_serialNumber);
    createParam(ADElcomat_focalLength_str, asynParamInt32, &index_focalLength);

    /* set some default values */
    setIntegerParam(index_connected, 0);
    setIntegerParam(index_serialNumber, 0);
    setIntegerParam(index_focalLength, 0);

    /* Initialize AD parameters */
    setIntegerParam(ADMaxSizeX, FRAME_WIDTH);
    setIntegerParam(ADMaxSizeY, 1);
    setIntegerParam(ADSizeX, FRAME_WIDTH);
    setIntegerParam(ADSizeX, 1);
    setIntegerParam(NDArraySizeX, FRAME_WIDTH);
    setIntegerParam(NDArraySizeX, 1);
    setIntegerParam(NDDataType, NDFloat64);
    setIntegerParam(ADImageMode, ADImageContinuous);
    setStringParam(ADManufacturer, "MÖLLER-WEDEL OPTICAL");

    strcpy(this->serialPortName, serialPortName);
    this->serialPortAddress = serialPortAddress;

    this->startEventId = epicsEventCreate(epicsEventEmpty);
    this->stopEventId = epicsEventCreate(epicsEventEmpty);

    goDisconnected();
    this->connect();
    thread.start();
}

/*
 * Destructor
 */
ADElcomat::~ADElcomat()
{
}

bool ADElcomat::startStreaming()
{
    size_t nwrite;
    asynStatus status = \
        pasynOctetSyncIO->write(serialPortUser, "A", 1, COMMAND_TIMEOUT,
            &nwrite);
    if (status != asynSuccess) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "Failed to read data point, error %d, nwrite %lu\n",
            status, nwrite);
        return false;
    }
    return true;
}

asynStatus ADElcomat::stop()
{
    size_t nwrite;
    return pasynOctetSyncIO->write(serialPortUser, "s", 1, COMMAND_TIMEOUT,
        &nwrite);
}

void ADElcomat::flush()
{
    pasynOctetSyncIO->flush(serialPortUser);
}

bool ADElcomat::getDeviceInfo()
{
    size_t nwrite;
    size_t nread;
    int eomReason;
    char rxBuffer[RX_BUFFER_SIZE];
    asynStatus status;
    rxBuffer[RX_BUFFER_SIZE - 1] = '\n';
    /* Read device configuration */
    status = pasynOctetSyncIO->writeRead(serialPortUser, "d", 1, rxBuffer,
        RX_BUFFER_SIZE - 1, COMMAND_TIMEOUT, &nwrite, &nread, &eomReason);
    if (status != asynSuccess) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "Failed to read device information, error %d, OEM reason %d\n",
            status, eomReason);
        return false;
    } else {
        int serialNumber, focalLength;
        bool ok = parseDeviceInfo(rxBuffer, &serialNumber, &focalLength);
        if (!ok)
            return false;
        setIntegerParam(index_serialNumber, serialNumber);
        setIntegerParam(index_focalLength, focalLength);
        callParamCallbacks();
    }
    return  true;
}

bool ADElcomat::connect()
{
    /* Connect to the serial port */
    asynStatus status = pasynOctetSyncIO->connect(this->serialPortName,
        this->serialPortAddress, &this->serialPortUser, NULL);

    bool result = true;
    if(status != asynSuccess)
    {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "Failed to connect to serial port=%s error=%d\n", serialPortName,
            status);
        setStringParam(ADStatusMessage, "Unable to connect");
        epicsThreadSleep(3.0);
        result = false;
    }

    /* Set proper terminators */
    pasynOctetSyncIO->setInputEos(
        this->serialPortUser, INPUT_EOS, strlen(INPUT_EOS));
    pasynOctetSyncIO->setOutputEos(
        this->serialPortUser, OUTPUT_EOS, strlen(OUTPUT_EOS));

    callParamCallbacks();
    return result;
}

bool ADElcomat::dataCallback(double x, double y, int flags)
{
    int imageCounter, numImagesCounter;
    int ndims = 1;
    size_t dims[1] = {FRAME_WIDTH};
    NDArray *pImage;
    epicsFloat64 data[FRAME_WIDTH] = {x, y};
    epicsTimeStamp timeStamp;
    epicsTimeGetCurrent(&timeStamp);
    pImage = this->pNDArrayPool->alloc(ndims, dims, NDFloat64,
        FRAME_WIDTH*sizeof(NDFloat64), NULL);
    pImage->uniqueId = imageCounter;
    pImage->timeStamp = timeStamp.secPastEpoch + timeStamp.nsec / 1e9;
    updateTimeStamp(&pImage->epicsTS);
    pImage->dataType = NDFloat64;
    pImage->ndims = ndims;
    pImage->dims[0].size = dims[0];
    pImage->dims[0].offset = 0;
    pImage->dims[0].binning = 0;
    pImage->pAttributeList->add("Flags", "Flags", NDAttrInt32, &flags);
    memcpy(pImage->pData, data, sizeof(data));
    getIntegerParam(NDArrayCounter, &imageCounter);
    getIntegerParam(ADNumImagesCounter, &numImagesCounter);
    imageCounter++;
    numImagesCounter++;
    setIntegerParam(NDArrayCounter, imageCounter);
    setIntegerParam(ADNumImagesCounter, numImagesCounter);
    callParamCallbacks();

    int arrayCallbacks;
    getIntegerParam(NDArrayCallbacks, &arrayCallbacks);
    if (arrayCallbacks) {
        unlock();
        doCallbacksGenericPointer(pImage, NDArrayData, 0);
        lock();
    }
    if (this->pArrays[0]) this->pArrays[0]->release();
    this->pArrays[0] = pImage;
    return true;
}

void ADElcomat::goDisconnected() {
    setStringParam(ADStatusMessage, "Disconnected");
    setIntegerParam(ADStatus, ADStatusDisconnected);
    setIntegerParam(ADAcquire, 0);
    callParamCallbacks();
    state = &ADElcomat::onDisconnected;
}

void ADElcomat::onDisconnected() {
    flush();
    int ok = getDeviceInfo();
    if (ok) {
        goIdle();
    } else {
        unlock();
        epicsEventWaitWithTimeout(startEventId, DISCONNECT_DELAY);
        lock();
    }
}

void ADElcomat::goIdle() {
    stop();
    setStringParam(ADStatusMessage, "Idle");
    setIntegerParam(ADStatus, ADStatusIdle);
    setIntegerParam(ADAcquire, 0);
    callParamCallbacks();
    state = &ADElcomat::onIdle;
}

void ADElcomat::onIdle() {
    unlock();
    epicsEventWait(this->startEventId);
    lock();
    int acquire;
    getIntegerParam(ADAcquire, &acquire);
    if (acquire)
        goAcquiring();
}

void ADElcomat::goAcquiring() {
    // Make sure detector is not running
    flush();
    if (!startStreaming()) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR, "Error starting acquision");
        return;
    }
    setStringParam(ADStatusMessage, "Acquiring");
    setIntegerParam(ADStatus, ADStatusAcquire);
    setIntegerParam(ADNumImagesCounter, 0);
    setIntegerParam(NDArrayCounter, 0);
    callParamCallbacks();
    state = &ADElcomat::onAcquiring;
}

void ADElcomat::onAcquiring() {
    int ok = processDataPoint();
    if (!ok) {
        if (++errorsCount >= 4) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "Acquisition aborted due to communication errors\n");
            errorsCount = 0;
            goDisconnected();
            return;
        }
    }
    unlock();
    epicsEventWaitStatus eventStatus = \
        epicsEventWaitWithTimeout(stopEventId, STOP_WAIT_TIMEOUT);
    lock();
    if (eventStatus == epicsEventWaitOK) {
        int acquire;
        getIntegerParam(ADAcquire, &acquire);
        if (!acquire)
            goIdle();
    }
}

/*
 * Override of the thread run virtual.  This is the function
 * that will be run for the thread.  Receive data from the device.
 */
void ADElcomat::run()
{
    lock();
    while (true) {
        // Possible states: onDisconnected (initial state), onIdle and
        //     onAcquiring
        // Transitions are triggered with functions: goDisconnected,
        //     goIdle and goAcquiring
        (this->*state)();
    }
}


bool ADElcomat::parseDeviceInfo(const char *text,
    int *serialNumber, int *focalLength)
{
    int nParsed = sscanf(
        text, "%d %*d %*d %*d %d", serialNumber, focalLength);
    if (nParsed != 2) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "Failed to parse device information: %s (%d)\n", text, nParsed);
        return false;
    }
    return true;
}


bool ADElcomat::parseDataPoint(char *text, double *x, double *y, int *flags)
{
    int nParsed = sscanf(text, "%*d %d %lf %lf", flags, x, y);
    if (nParsed != 3) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "Failed to parse data point: %s (%d)\n", text, nParsed);
        return false;
    }
    return true;
}

bool ADElcomat::processDataPoint()
{
    int eomReason, flags;
    size_t nread;
    double x, y;
    char rxBuffer[RX_BUFFER_SIZE];
    rxBuffer[RX_BUFFER_SIZE - 1] = '\n';

    asynStatus status = pasynOctetSyncIO->read(serialPortUser, rxBuffer,
        RX_BUFFER_SIZE - 1, COMMAND_TIMEOUT, &nread, &eomReason);

    if (status != asynSuccess) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "Failed to read data point, error %d, OEM reason %d, "
            "buffer %s, nread %lu\n", status, eomReason, rxBuffer, nread);
        return false;
    }

    if(!parseDataPoint(rxBuffer, &x, &y, &flags))
        return false;

    bool ok = this->dataCallback(x, y, flags);
    callParamCallbacks();
    return ok;
}

asynStatus ADElcomat::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    asynStatus status = asynSuccess;

    /* Any work we need to do */
    int parameter = pasynUser->reason;

    if (parameter == ADAcquire)
    {
        int adstatus;
        getIntegerParam(ADStatus, &adstatus);
        bool shouldAcquire = value && adstatus == ADStatusIdle;
        if (shouldAcquire) {
            setIntegerParam(ADAcquire, 1);
            epicsEventSignal(this->startEventId);
        } else if (!value) {
            setIntegerParam(ADAcquire, 0);
            epicsEventSignal(this->stopEventId);
        }
    } else if (parameter == ADImageMode) {
        // we only allow continuous mode at the moment, so, don't update it
    } else if (parameter == NDDataType) {
        // we only allow NDFloat64 at the moment, so, don't update it
    } else {
        status = ADDriver::writeInt32(pasynUser, value);
    }

    callParamCallbacks();
    return status;
}

/** Configuration command, called directly or from iocsh */
extern "C" int ADElcomatConfig(const char* portName, const char* serialPortName,
        int serialPortAddress, int maxBuffers, size_t maxMemory, int priority,
        int stackSize)
{
    new ADElcomat(portName, serialPortName, serialPortAddress, maxBuffers,
        maxMemory, priority, stackSize);
    return(asynSuccess);
}

/** Code for iocsh registration */
static const iocshArg ADElcomatConfigArg0 = {"Port name", iocshArgString};
static const iocshArg ADElcomatConfigArg1 = {"Serial port name", iocshArgString};
static const iocshArg ADElcomatConfigArg2 = {"Serial port address", iocshArgInt};
static const iocshArg ADElcomatConfigArg3 = {"maxBuffers", iocshArgInt};
static const iocshArg ADElcomatConfigArg4 = {"maxMemory", iocshArgInt};
static const iocshArg ADElcomatConfigArg5 = {"priority", iocshArgInt};
static const iocshArg ADElcomatConfigArg6 = {"stackSize", iocshArgInt};
static const iocshArg* const ADElcomatConfigArgs[] =  {&ADElcomatConfigArg0,
                                                         &ADElcomatConfigArg1,
                                                         &ADElcomatConfigArg2,
                                                         &ADElcomatConfigArg3,
                                                         &ADElcomatConfigArg4,
                                                         &ADElcomatConfigArg5,
                                                         &ADElcomatConfigArg6};
static const iocshFuncDef configADElcomat = {"ADElcomatConfig", 7, ADElcomatConfigArgs};
static void configADElcomatCallFunc(const iocshArgBuf *args)
{
    ADElcomatConfig(args[0].sval, args[1].sval, args[2].ival, args[3].ival,
        args[4].ival, args[5].ival, args[6].ival);
}

static void ADElcomatRegister(void)
{
    iocshRegister(&configADElcomat, configADElcomatCallFunc);
}

extern "C" { epicsExportRegistrar(ADElcomatRegister); }


