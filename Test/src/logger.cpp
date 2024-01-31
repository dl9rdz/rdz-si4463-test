#include "logger.h"
#include "colors.h"
#include <Arduino.h>
#include <stdarg.h>

// Simple configurable logger

static uint16_t logmask;

const char *defcol=CRESET;

static const char *color(uint16_t mask) {
	if(mask&LOG_INFO)  return BBLK;
	if(mask&LOG_RADIO) return CYN;
	if(mask&LOG_RXFRM) return RED;
	if(mask&LOG_RXTLM) return GRN;
	if(mask&LOG_RXRAW) return BLU;
	if(mask&LOG_RXDBG) return MAG;
	if(mask&LOG_SPI)   return YEL;
	return defcol;
}

void logSetMask(uint16_t mask) {
	logmask = mask;
	Serial.printf("Logmask set to %0x\n", mask);
}

int logEnabled(uint16_t type) {
	return !!(logmask & type);
}

void logPrint(uint16_t type, const char *format, ...)
{
        va_list args;
        va_start(args, format);
        if( type & logmask ) {
                char buf[256];
		Serial.print( color(type) );
                vsnprintf( buf, 256, format, args );
                Serial.print( buf );
		Serial.print( color(0) );
        }
        va_end(args);
}
        

