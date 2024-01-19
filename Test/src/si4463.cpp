#include <Arduino.h>
#include <SPI.h>
// Just

#include "si4463.h"
#include "radio_config_Si4463.h"

#define HSPI_SCLK 12
#define HSPI_MISO 13
#define HSPI_MOSI 11
#define HSPI_CS 10
#define SDN 1

uint8_t buf[] = { 0x13, 0x00, 0x00, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t power[] = { 0x02, 0x01, 0x00, 0x01, 0xc9, 0xc3, 0x80  };
uint8_t partinfo[] = { 0x01 };

uint8_t Radio_Configuration_Data_Array[] = RADIO_CONFIGURATION_DATA_ARRAY;


SPIClass *hspi;


// Low-level SPI functions

#define DUMMY 0xFF
void spi_getresponse(SPIClass *hspi, uint8_t *buf, int len) {
    Serial.print("spi_getreponse: ");
    for(int i=0; i<len; i++) {
        buf[i] = hspi->transfer(DUMMY);
        Serial.printf("%02x ", buf[i]);
    }
    Serial.println("");
}
void spi_sendcmd(SPIClass *hspi, uint8_t *buf, int len) {
    Serial.print("spi_sendcmd: ");
    uint8_t dummy[len];
    for(int i=0; i<len; i++) {
        dummy[len] = hspi->transfer(buf[i]);
        Serial.printf("%02x>%02x ", buf[i], dummy[i]);
    }
    Serial.println();
}

// Radio chip functions
static uint8_t ctsOK = 0;

void si4463_init() {
    delay(2000); // Make sure serial terminal is ready
    // Init library
    Serial.println("si4463_init: Initializing SPI bus");
    hspi = new SPIClass(HSPI);
    hspi->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, -1); //SCLK, MISO, MOSI, SS

    digitalWrite(HSPI_CS, HIGH);
    digitalWrite(SDN, LOW);

    pinMode(HSPI_CS, OUTPUT);  // CnS
    pinMode(SDN, OUTPUT);  // CnS
}

void si4463_poweron() {
    digitalWrite(SDN, LOW);
}
void si4463_poweroff() {
    digitalWrite(SDN, HIGH);
}
void si4463_reset() {
    Serial.println("Si4463 power off/on reset");
    digitalWrite(SDN, HIGH);
    delay(500);
    digitalWrite(SDN, LOW);
    delay(500);
    ctsOK = 0;  // need to wait for CTS
}
  
void si4463_sendrecv(uint8_t *cmd, int cmdlen, uint8_t *resp, int resplen) {
    uint8_t ctsval;
    hspi->beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));

    while(!ctsOK) {
        printf("si4463_sendrecv: Waiting for CTS");
        digitalWrite(HSPI_CS, LOW);
        buf[0] = 0x44; buf[1] = 0xFF;
        for(int i=0; i<2; i++) {
            buf[i] = hspi->transfer(buf[i]);
        }
        Serial.printf("Sending 0x44 0xFF, getting back: 0x%02X 0x%02X\n", buf[0], buf[1]);
        if(buf[1]==0xFF) break;  // OK
        digitalWrite(HSPI_CS, HIGH);
        Serial.println("Waiting...\n");
        delay(500);  // should be shorter
    }

    // Send command
    Serial.print("Sending: ");
    for(int i=0; i<cmdlen; i++) { Serial.printf("%02x ", cmd[i]); }
    digitalWrite(HSPI_CS, LOW);
    spi_sendcmd(hspi, cmd, cmdlen);
    digitalWrite(HSPI_CS, HIGH);

    // 
    digitalWrite(HSPI_CS, LOW);
    while(1) { // wait until CTS
        hspi->transfer(0x44);   // read CMD buffer
        ctsval = hspi->transfer(DUMMY);
        if(ctsval==0xFF) { 
            Serial.println("Got CTS");
            if(resplen>0) {
		Serial.printf("Reading response (%d bytes)\n", resplen);
                spi_getresponse(hspi, resp, resplen);
		for(int i=0; i<resplen; i++) { Serial.printf("%02X ", resp[i]); }
		Serial.println("");
            }
            digitalWrite(HSPI_CS, HIGH);
            break;
        }
        digitalWrite(HSPI_CS, HIGH);
        // TODO: Maybe add timeout and error
        delay(1000);
        Serial.println("CTSWAIT");
    }
    if(ctsval==0xff) {  // should always be the case!
        ctsOK = 1;
    }
    hspi->endTransaction();
    Serial.println("OK");
}


// Send all configuration commands from Radio_Configuration_Data_Array
// (created using Wireless Development Suite)
int si4463_configure() {
    Serial.println("Configure() called");
    uint8_t *initdata = Radio_Configuration_Data_Array;
    uint8_t rbuf[20];
    while (*initdata != 0) {
        uint8_t numbytes = *initdata++;
        if(numbytes>16) {
            Serial.printf("Number of bytes in init sequence exceeds 16: %d\n", numbytes);
            return -1;
        }
        si4463_sendrecv(initdata, numbytes, rbuf, 0);
        initdata += numbytes;
    }
    return 0;
}



int spi_receive(int waitcts, uint8_t *retbuf, int retlen)
{
    uint8_t buf[2];

    if(waitcts) {
        digitalWrite(HSPI_CS, HIGH);
        while(1) {
            delay(100);
            printf("Waiting for CTS");
            digitalWrite(HSPI_CS, LOW);
            buf[0] = 0x44; buf[1] = 0xFF;
            for(int i=0; i<2; i++) {
                buf[i] = hspi->transfer(buf[i]);
            }
            Serial.printf("Sending 0x44 0xFF, getting back: 0x%02X 0x%02X\n", buf[0], buf[1]);
            if(buf[1]==0xFF) break;  // OK
            digitalWrite(HSPI_CS, HIGH);
            Serial.println("Waiting...\n");
            delay(500);  // should be shorter
        }
    }
    if(retlen>0) {
         spi_getresponse(hspi, retbuf, retlen);
    }
    digitalWrite(HSPI_CS, HIGH);
    Serial.printf("Response: (retlen=%d)\n", retlen);
    for(int i=0; i<retlen; i++) {
        Serial.printf("%02x ", retbuf[i]);
    }
    Serial.println("---");
    return 0;
}



// "Public API" -> these do SPI transaction (the others assume to be within one?) 

/* wait for CTS (if waitcts is set), then read response */
int si4463_receive(int waitcts, uint8_t *retbuf, int retlen)
{
    hspi->beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));
    int retval = spi_receive(waitcts, retbuf, retlen);
    hspi->endTransaction();
    return retval;
}

void si4463_waitcts() {
    si4463_receive(1, (uint8_t*)0, 0);
}    

void si4463_writedata(uint8_t *cmd, int cmdlen)
{
    hspi->beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));
    digitalWrite(HSPI_CS, LOW);
    spi_sendcmd(hspi, cmd, cmdlen);
    digitalWrite(HSPI_CS, HIGH);
    hspi->endTransaction();
}
     

#if 0
void spi_sendrecv(SPIClass *hspi, uint8_t *cmd, int cmdlen, uint8_t *retbuf, int retlen)
{
    Serial.print("Sending: ");
    for(int i=0; i<cmdlen; i++) { Serial.printf("%02x ", cmd[i]); }
    Serial.println("");
    hspi->beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));
    digitalWrite(HSPI_CS, LOW);
    spi_sendcmd(hspi, cmd, cmdlen);

    // Fast commands can continue immediately, others needs to wait for CTS
    switch (cmd[0]) {
        case 0x44: // READ_CMD_BUFF
        case 0x50: // FRR_A_READ
        case 0x51: // FRR_B_READ
        case 0x53: // FRR_C_READ
        case 0x57: // FRR_D_READ
        case 0x66: // WRITE_TX_FIFO
        case 0x77: // READ_RX_FIFO
                break;
        default:
          digitalWrite(HSPI_CS, HIGH);
          // wait for CTS
          while(1) {
             delay(100);
             digitalWrite(HSPI_CS, LOW);
             buf[0] = 0x44; buf[1] = 0xFF;
             uint8_t b[2];
             for(int i=0; i<2; i++) {
                 b[i] = hspi->transfer(buf[i]);
             }
             Serial.print("Sending 0x44 0xFF, getting back: ");
             for(int i=0; i<2; i++) {
                Serial.printf("%02x ", b[i]);
             }
             Serial.println("");
             if(b[1]==0xFF) break;  // OK
             digitalWrite(HSPI_CS, HIGH);
             Serial.println("Waiting...\n");
             delay(500);
          }
    }
    spi_getresponse(hspi, retbuf, retlen);
    Serial.print("Response: ");
    for(int i=0; i<retlen; i++) {
        Serial.printf("%02x ", retbuf[i]);
    }
    Serial.println("");
    digitalWrite(HSPI_CS, HIGH);
    hspi->endTransaction();
}

#else
void spi_sendrecv(SPIClass *hspi, uint8_t *cmd, int cmdlen, uint8_t *retbuf, int retlen)
{
    Serial.print("sendrecv: Sending: ");
    for(int i=0; i<cmdlen; i++) { Serial.printf("%02x ", cmd[i]); }
    Serial.println("");
    hspi->beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));
    digitalWrite(HSPI_CS, LOW);
    spi_sendcmd(hspi, cmd, cmdlen);

    // Fast commands can continue immediately, others needs to wait for CTS
    int waitcts = 1;
    switch (cmd[0]) {
        case 0x44: // READ_CMD_BUFF
        case 0x50: // FRR_A_READ
        case 0x51: // FRR_B_READ
        case 0x53: // FRR_C_READ
        case 0x57: // FRR_D_READ
        case 0x66: // WRITE_TX_FIFO
        case 0x77: // READ_RX_FIFO
            waitcts = 0;
            break;
    }
    Serial.printf("receiving, waitcts=%d\n", waitcts);
    spi_receive(waitcts, retbuf, retlen);
    Serial.println("");
    digitalWrite(HSPI_CS, HIGH);
    hspi->endTransaction();
}
#endif

void si4463_test() {
    digitalWrite(HSPI_CS, HIGH);
    digitalWrite(SDN, LOW);
    pinMode(HSPI_CS, OUTPUT);  // CnS
    pinMode(SDN, OUTPUT);  // CnS

    uint8_t b[sizeof(buf)];
    Serial.println("Starting up");
    delay(2000);
    int i=0;
    long int x=0;
    {
       Serial.println("Poweroff for 2s");
       si4463_poweroff();

       Serial.println("Poweron");
       si4463_poweron();
       delay(100);

       si4463_waitcts();
       delay(2000);

       uint8_t res;
       spi_sendrecv(hspi, power, sizeof(power), &res, 1);
       Serial.printf("Power ok: %x\n", res);
       delay(400);

       //uint8_t rbuf[10];
       //si4463_partinfo((st_partinfo *)rbuf);
       uint8_t rbuf[10];
       spi_sendrecv(hspi, partinfo, 1, rbuf, 9);
    }
    Serial.printf("Test done\n");
}

unsigned short swaps( unsigned short val)
{
    return ((val & 0xff) << 8) | ((val & 0xff00) >> 8);
}

int si4463_partinfo(st_partinfo *pi) {
    Serial.println("Requesting partinfo");
    si4463_sendrecv(partinfo, 1, (uint8_t *)pi, 9);
    pi->part = swaps(pi->part);
    pi->id = swaps(pi->id);
    return 0;
}



