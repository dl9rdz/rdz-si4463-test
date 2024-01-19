
#include "src/si4463.h"


void setup() {
    Serial.begin(115200);
    delay(2000);     // Just to get all log messages on the serial port :)

    si4463_init();   // Initialize SPI bus
    si4463_reset();  // Power-cycle the radio chip
    si4463_configure();   // Send all configuration stuff from  Wireless Development Suite
}

void loop() {
    Serial.println("si4463 partinfo:");
    st_partinfo pi;
    int res = si4463_partinfo(&pi);
    Serial.printf("Partinfo: Chip %x, part %x, pbuild %x, id %x, customer %x, romid %x\n",
	pi.chiprev, pi.part, pi.pbuild, pi.id, pi.customer, pi.romid);

    delay(20000);
}
