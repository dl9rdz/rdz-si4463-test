#include "logger.h"
#include "colors.h"
#include <Arduino.h>
#include <stdarg.h>

// Simple configurable logger

static uint16_t logmask;
static uint8_t useColor = 0;

const char *defcol=CRESET;

static const char *color(uint16_t mask) {
	if(!useColor) return "";
	if(mask&LOG_INFO)  return BBLK;
	if(mask&LOG_RADIO) return CYN;
	if(mask&LOG_RXFRM) return RED;
	if(mask&LOG_RXTLM) return GRN;
	if(mask&LOG_RXRAW) return BLU;
	if(mask&LOG_RXDBG) return MAG;
	if(mask&LOG_SPI)   return YEL;
	return defcol;
}

void logSetColor(uint8_t onoff) {
	useColor = onoff;
	if(!useColor) Serial.print( defcol ); // make sure color is off
}

void logSetMask(uint16_t mask) {
	logmask = mask;
	Serial.printf("Logmask set to %0x\n", mask);
}
uint16_t logGetMask() {
	return logmask;
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
		if(useColor)
			Serial.print( color(type) );
                vsnprintf( buf, 256, format, args );
                Serial.print( buf );
		if(useColor)
			Serial.print( defcol );
        }
        va_end(args);
}
        

