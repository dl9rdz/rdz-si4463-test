#include <inttypes.h>

void logPrint(uint16_t type, const char *format, ...);

void logSetMask(uint16_t mask);
uint16_t logGetMask();
int logEnabled(uint16_t mask);

void logSetColor(uint8_t onoff);

#define LOG_INFO  (1<<0)
#define LOG_RADIO (1<<1)
#define LOG_RXFRM (1<<2)
#define LOG_RXTLM (1<<3)
#define LOG_RXRAW (1<<4)
#define LOG_RXSTAT (1<<5)
#define LOG_RXDBG (1<<6)
#define LOG_SPI   (1<<7)
