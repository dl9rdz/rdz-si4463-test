#include "logger.h"
#include <Arduino.h>
#include <stdarg.h>

// Simple configurable logger

static uint16_t logmask;

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
                vsnprintf( buf, 256, format, args );
                Serial.print( buf );
        }
        va_end(args);
}
        

