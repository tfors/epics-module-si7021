/* drvAsynSi7021.cpp
 *
 * EPICS asynPortDriver for Si7021 Digital Temperature
 * and Humidity sensor under Linux using i2c-dev.
 *
 * Thomas Fors <tom@fors.net>
 * December, 2017
 */

#include <epicsExport.h>
#include <epicsThread.h>
#include <iocsh.h>

#include "drvAsynSi7021.h"

static const char* driverName = "drvAsynSi7021";

static void pollTask(void* drvPvt);

drvAsynSi7021::drvAsynSi7021(const char* portName, int i2cPortNum, int i2cAddr)
    : asynI2CDriver(portName, i2cPortNum,
                    1, /* maxAddr */
                       /* Interface mask */
                    (asynFloat64Mask | asynDrvUserMask),
                       /* Interrupt mask */
                    (asynFloat64Mask),
                    0, /* asynFlags (does not block and is not multi-device) */
                    1, /* Autoconnect */
                    0, /* Default priority */
                    0) /* Default stack size */
{
    const char* functionName = "drvAsynLSM303D";

    this->i2cAddr = (unsigned short)i2cAddr;

    createParam(P_TempCString, asynParamFloat64, &P_Temp_C);
    createParam(P_TempFString, asynParamFloat64, &P_Temp_F);
    createParam(P_HumString, asynParamFloat64, &P_Hum);

    eventId_ = epicsEventCreate(epicsEventEmpty);
    if (epicsThreadCreate("drvAsynSi7021Task", epicsThreadPriorityMedium,
                          epicsThreadGetStackSize(epicsThreadStackMedium),
                          (EPICSTHREADFUNC)::pollTask, this) == NULL) {
        printf("%s:%s: epicsThreadCreate failure\n", driverName, functionName);
        return;
    }

    // Call our connect method to work-around the fact that autoconnect flag
    // triggers the base class to call connect before our constructor runs.
    this->connect(this->pasynUserSelf);
}

asynStatus drvAsynSi7021::connect(asynUser* pasynUser)
{
    asynStatus status = asynSuccess;

    /* Perform i2c connect */
    status = this->i2c_connect(pasynUser);
    if (status != asynSuccess) {
        return status;
    }

    /* Reset Si7021 */
    if (i2c_wr(i2cAddr, (unsigned char*)"\xfe", 1) != 0) {
        return asynError;
    }
    epicsThreadSleep(0.015);

    return status;
}

asynStatus drvAsynSi7021::disconnect(asynUser* pasynUser)
{
    /* Perform i2c disconnect */
    return this->i2c_disconnect(pasynUser);
}

static void pollTask(void* drvPvt)
{
    drvAsynSi7021* pPvt = (drvAsynSi7021*)drvPvt;
    pPvt->pollTask();
}

void drvAsynSi7021::pollTask(void)
{
    epicsTimeStamp now;
    epicsUInt32 delay_ns;
    unsigned char vals[3];
    epicsFloat64 newVal;
    epicsFloat64 smoo = 0.983;

    lock();
    while (1) {
        unlock();

        epicsTimeGetCurrent(&now);
        delay_ns = 50000000 - (now.nsec % 50000000); /* 20 Hz */
        epicsEventWaitWithTimeout(eventId_, delay_ns / 1.e9);

        lock();

        /* Make RH measurement */
        i2c_wr(i2cAddr, (unsigned char*)"\xf5", 1);
        epicsThreadSleep(0.02);
        i2c_rd(i2cAddr, vals, 3);
        newVal = (vals[0] << 8) | vals[1];
        newVal = 125 * newVal / 65536 - 6;
        if (hum == 0) {
            hum = newVal;
        } else {
            hum = (1 - smoo) * newVal + smoo * hum;
        }
        setDoubleParam(P_Hum, hum);

        /* Read the assocuated temperature measurement */
        i2c_wr_rd(i2cAddr, (unsigned char*)"\xe0", 1, vals, 2);
        newVal = (vals[0] << 8) | vals[1];
        newVal = 175.72 * newVal / 65536 - 46.85;
        if (temp == 0) {
            temp = newVal;
        } else {
            temp = (1 - smoo) * newVal + smoo * hum;
        }
        setDoubleParam(P_Temp_C, temp);
        setDoubleParam(P_Temp_F, temp * 9 / 5 + 32);

        updateTimeStamp();
        callParamCallbacks();
    }
}

extern "C" {

int drvAsynSi7021Configure(const char* portName, int i2cPortNum, int i2cAddr)
{
    new drvAsynSi7021(portName, i2cPortNum, i2cAddr);
    return (asynSuccess);
}

/* EPICS iocsh shell commands */
static const iocshArg initArg0          = { "portName", iocshArgString };
static const iocshArg initArg1          = { "i2c port num", iocshArgInt };
static const iocshArg initArg2          = { "i2c address", iocshArgInt };
static const iocshArg* const initArgs[] = { &initArg0, &initArg1, &initArg2 };
static const iocshFuncDef initFuncDef
    = { "drvAsynSi7021Configure", 3, initArgs };

static void initCallFunc(const iocshArgBuf* args)
{
    drvAsynSi7021Configure(args[0].sval, args[1].ival, args[2].ival);
}

void drvAsynSi7021Register(void) { iocshRegister(&initFuncDef, initCallFunc); }

epicsExportRegistrar(drvAsynSi7021Register);
}
