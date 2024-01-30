#include "logger.h"
#include <Arduino.h>
#include <stdarg.h>

// Simple configurable logger

static uint16_t logmask;

const char *defcol="\e[0;30m";  //black
static const char *color(uint16_t mask) {
	if(mask&LOG_RXRAW) return "\e[0;34m";  // blue
	if(mask&LOG_RXFRM) return "\e[0;31m";  // red
	if(mask&LOG_RXTLM) return "\e[43m";    //yellow bg
	if(mask&LOG_RXDBG) return "\e[0;35m";  // mag
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
        

