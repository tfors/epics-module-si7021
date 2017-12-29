/* drvAsynSi7021.h
 *
 * EPICS asynPortDriver for Si7021 Digital Temperature
 * and Humidity sensor under Linux using i2c-dev.
 *
 * Thomas Fors <tom@fors.net>
 * December, 2017
 */

#include <epicsEvent.h>

#include "asynI2CDriver.h"

#define P_TempCString "Temperature(C)" /* asynFloat64 */
#define P_TempFString "Temperature(F)" /* asynFloat64 */
#define P_HumString "Humidity"         /* asynFloat64 */

class drvAsynSi7021 : public asynI2CDriver {

public:
    drvAsynSi7021(const char* portName, int i2cPortNum, int i2cAddr);

    virtual asynStatus connect(asynUser* pasynUser);
    virtual asynStatus disconnect(asynUser* pasynUser);

    void pollTask(void);

protected:
    int P_Temp_C;
    int P_Temp_F;
    int P_Hum;

private:
    epicsEventId eventId_;
    unsigned short i2cAddr;

    double temp;
    double hum;

    int read_reg(unsigned char reg, unsigned char* value, unsigned short len);
    int write_reg(unsigned char reg, unsigned char value);
};
