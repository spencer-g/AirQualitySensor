//PMS5003 Library

#ifndef pms5003_h
#define pms5003_h

#if defined(ARDUINO) && (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#ifndef SOFTWARE_SERIAL
#include <SoftwareSerial.h>
#define SOFTWARE_SERIAL
#endif

#define PMSLIB_OK	0
#define PMSLIB_ERROR_TIMEOUT	-1
#define PMSLIB_ERROR_CHECKSUM	-2

class pms5003
{
public:
	void setPins(SoftwareSerial*);
	int8_t sleep();
	int8_t wakeup();
	int8_t activeRead();
	int8_t passiveRead();
    int8_t read();
	int pm_1_0;
	int pm_2_5;
	int pm_10;
	int pm_1_0_ae;
	int pm_2_5_ae;
	int pm_10_ae;
	int bin_0_3;
	int bin_0_5;
	int bin_1_0;
	int bin_2_5;
	int bin_5_0;
	int bin_10;
	SoftwareSerial* mySerial;
private:
	int intConcat(byte, byte);
};
#endif
//
// END OF FILE
//