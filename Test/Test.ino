
#include "src/si4463.h"


void setup() {
    Serial.begin(115200);
    delay(2000);     // Just to get all log messages on the serial port :)

    si4463_init();   // Initialize SPI bus
    si4463_reset();  // Power-cycle the radio chip
    si4463_configure();   // Send all configuration stuff from  Wireless Development Suite
}

void loop() {
    static int blubb = 0;
    Serial.println("si4463 partinfo:");
    st_partinfo pi;
    int res = si4463_partinfo(&pi);
    Serial.printf("Partinfo: Chip %x, part %x, pbuild %x, id %x, customer %x, romid %x\n",
	pi.chiprev, pi.part, pi.pbuild, pi.id, pi.customer, pi.romid);

    // receive immediately on ch35 (403.5 MHz)
    while(1) {
        si4463_startrx(35, 0, 20/*len*/, 8, 8, 8);
        delay(1000);
        int n = si4463_getfifoinfo();
        if(n==0) {
            if( ((++blubb)&16) ==0) Serial.println("Empty fifo");
            delay(100);
	    continue;
	}
        uint8_t buf[n];
	si4463_readfifo(buf, n);
	Serial.print("FIFO: ");
        for(int i=0; i<n; i++) { Serial.printf("%02x ", buf[i]); }
        Serial.println("");
    }
    delay(20000);
}
