#include <Arduino.h>
#include <SPI.h>
// Just

#include "logger.h"
#include "si4463.h"
#include "radio_config_Si4463.h"
//#include "radio_config_Si4463_dfm.h"
//#include "radio_config_Si4463_rs41.h"

#define HSPI_SCLK 12
#define HSPI_MISO 13
#define HSPI_MOSI 11
#define HSPI_CS 10
#define SDN 1

uint8_t buf[] = { 0x13, 0x00, 0x00, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t power[] = { 0x02, 0x01, 0x00, 0x01, 0xc9, 0xc3, 0x80  };
uint8_t partinfo[] = { 0x01 };

uint8_t Radio_Configuration_Data_Array[] = RADIO_CONFIGURATION_DATA_ARRAY;
uint8_t Radio_Patches_Powerup_Array[] = RADIO_PATCHES_POWERUP_ARRAY;


SPIClass *hspi;


/******************************************************************************
 * Low-level SPI functions
 ******************************************************************************/

#define DUMMY 0xFF

void spi_getresponse(SPIClass *hspi, uint8_t *buf, int len) {
	//Serial.print("spi_getreponse: ");
	for(int i=0; i<len; i++) {
		buf[i] = hspi->transfer(DUMMY);
		//Serial.printf("%02x ", buf[i]);
	}
	//Serial.println("");
}

void spi_sendcmd(SPIClass *hspi, const uint8_t *buf, int len) {
	//Serial.print("spi_sendcmd: ");
	uint8_t dummy[len];
	for(int i=0; i<len; i++) {
		dummy[len] = hspi->transfer(buf[i]);
		//Serial.printf("%02x>%02x ", buf[i], dummy[i]);
	}
	//Serial.println();
}


// Radio chip functions
static uint8_t ctsOK = 0;

void si4463_init() {
	//test
#if 0
	for(int i=0; i<100; i++){
 	int p = 10 + (i&3);
		pinMode(p, OUTPUT);
		digitalWrite(p, HIGH);
 	delay(1000);
		digitalWrite(p, LOW);
 	delay(1000);
	}
#endif

	delay(2000); // Make sure serial terminal is ready
	// Init library
	logPrint(LOG_SPI, "si4463_init: Initializing SPI bus\n");
	hspi = new SPIClass(HSPI);
	hspi->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, -1); //SCLK, MISO, MOSI, SS

	digitalWrite(HSPI_CS, HIGH);
	digitalWrite(SDN, LOW);

	pinMode(HSPI_CS, OUTPUT);  // CnS
	pinMode(SDN, OUTPUT);  // CnS
}

void si4463_waitcts(); // further below...

void spi_sendrecv(SPIClass *hspi, uint8_t *cmd, int cmdlen, uint8_t *retbuf, int retlen);
void si4463_sendrecv(const uint8_t *cmd, int cmdlen, uint8_t *resp, int resplen);

// This works for powering ip.  
// Doing poweron() followed by power_up() does not. Need to check...
void _test() {
        uint8_t b[sizeof(buf)];
#if 1
        Serial.println("Starting up");
        delay(2000);
#endif
        int i=0;
        long int x=0;
#if 1
        {
           
           uint8_t res;
           spi_sendrecv(hspi, power, sizeof(power), &res, 1);
           Serial.printf("Power ok: %x\n", res);
           delay(400);
        }
#endif
        Serial.printf("Test done\n");
}       

// Important note on powerup sequence:
// https://community.silabs.com/s/article/si4x6x-c2a-si4x55-c2a-startup-sequence
// First SPI transaction after SDN low has to finish in less than 4ms.
// (The waitcts is a SPI transaction...)
void si4463_poweron() {
	digitalWrite(SDN, LOW);
	ctsOK = 0;
	Serial.println("Poweron: Waiting for CTS");
	si4463_waitcts();
	   uint8_t res[40];
	   delay(2000);
	   //si4463_sendrecv(power, sizeof(power), res, 30);
           spi_sendrecv(hspi, power, sizeof(power), res, 1);
	   //si4463_power_up_cmd();
	   //si4463_power_up_cmd();
	   //_test();
	   Serial.printf("Power ok: %x\n", res[0]);
	   delay(2200);
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
// as in test:::
	   delay(100);
	   si4463_waitcts();
}

void si4463_sendrecv(const uint8_t *cmd, int cmdlen, uint8_t *resp, int resplen) {
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
	logPrint(LOG_RXDBG, "Sending: ");
	for(int i=0; i<cmdlen; i++) { logPrint(LOG_RXDBG, "%02x ", cmd[i]); }
	digitalWrite(HSPI_CS, LOW);
	spi_sendcmd(hspi, cmd, cmdlen);
	digitalWrite(HSPI_CS, HIGH);

	// min 80 ns
	delay(1); 
	digitalWrite(HSPI_CS, LOW);
	while(1) { // wait until CTS
		hspi->transfer(0x44);   // read CMD buffer
		ctsval = hspi->transfer(DUMMY);
		if(ctsval==0xFF) { 
			logPrint(LOG_RXDBG, " [CTS OK] ");
			if(resplen>0) {
				logPrint(LOG_RXDBG, "Response (%d bytes): ", resplen);
				spi_getresponse(hspi, resp, resplen);
				for(int i=0; i<resplen; i++) { logPrint(LOG_RXDBG, "%02X ", resp[i]); }
				logPrint( LOG_RXDBG, "\n");
			}
			digitalWrite(HSPI_CS, HIGH);
			break;
		}
		digitalWrite(HSPI_CS, HIGH);
		// TODO: Maybe add timeout and error
		delay(100);
		logPrint( LOG_RXDBG, "CTSWAIT\n");
	}
	if(ctsval==0xff) {  // should always be the case!
		ctsOK = 1;
	}
	hspi->endTransaction();
}


/*******************************************************************************
 * Configuration commands
 */

int si4463_send_config(uint8_t *initdata) {
	Serial.println("Configure() called");
	uint8_t rbuf[20];
        Serial.printf("\nSendconfig: *initdata is %d", *initdata);
	while (*initdata != 0) {
		uint8_t numbytes = *initdata++;
		if(numbytes>16) {
			Serial.printf("Number of bytes in init sequence exceeds 16: %d\n", numbytes);
			return -1;
		}
		si4463_sendrecv(initdata, numbytes, rbuf, 0);
		initdata += numbytes;
        	Serial.printf("\nSendconfig: *initdata is %d", *initdata);
	}
	return 0;
}
// Send all configuration commands from Radio_Configuration_Data_Array
// (created using Wireless Development Suite)
int si4463_configure() {
	uint8_t *initdata = Radio_Configuration_Data_Array;
	return si4463_send_config(initdata);
}

// Send patches (created using WDS) and RF_POWER_UP command
int si4463_power_up_cmd() {
	uint8_t *initdata = Radio_Patches_Powerup_Array;
	return si4463_send_config(initdata);
}
  

float basefreq = 400000000;   // TODO: set via config
float chanstep = 100000;      // TODO: set via config
static uint8_t channel;
static int16_t freqoffset;

void si4463_setchannel(uint8_t ch) {
	channel = ch;
}	
// offset is 2^19 * outdiv * offset_in_Hz / (Npresc * freq_xo)
int si4463_setfreqoffset(uint16_t offset) {
	freqoffset = offset;
	uint8_t buf[6] = { 0x11, 0x20, 2, 0x0d, 0, 0 };  // set property, group 0x20, 2 properties, starting at 0x0d
	buf[4] = offset / 256;
	buf[5] = offset & 0xFF;
	si4463_sendrecv(buf, 6, (uint8_t *)0, 0);
	return 0;
}

// Let's set the frequency directly via FREQ_CONTROL
// RF(Hz) = (F_c_inte + F_C_frac / 2^19 ) * (N_PRESC * freq_xo / outdiv )
// OUTDIV: The sample config uses MODEM_CLKGEN_BAND = 0x09 (FORCE_SY_RECAL=0, SY_SEL_1, BAND=1 == FVCO_DIV_6)
// However, the data sheet(https://www.silabs.com/documents/public/data-sheets/Si4463-61-60-C.pdf) page 31 says
// outdiv = 10 for 350..420 MHz. Seems that outdiv=10 is right.
//
// see API document at http://www.silabs.com/documents/public/application-notes/EZRadioPRO_REVC2_API.zip
//
// Comment from silabs:
// The error is in the API document, probably because that list was taken from the REVB1 API without
// applying the change of REVC2. Sorry for the inconvenience.
//
// The correct list is
// FVCO_DIV_4 0 Output is FVCO/4.
// FVCO_DIV_10 1 Output is FVCO/10.
// FVCO_DIV_8 2 Output is FVCO/8.
// FVCO_DIV_12 3 Output is FVCO/12.
// FVCO_DIV_16 4 Output is FVCO/16.
// FVCO_DIV_24 5 Output is FVCO/24.
// FVCO_DIV_24_2 6 Output is FVCO/24.
// FVCO_DIV_24_3 7 Output is FVCO/24.

#define OUTDIV 10
#define NPRES 2
#define FREQOSC 26000000
void si4463_setfreq(float f) {
	float plldiv = f / NPRES / FREQOSC * OUTDIV;

	int intpart = (int)plldiv - 1;
	plldiv = plldiv - intpart;
	uint32_t frac = (uint32_t)((float)(1<<19) * plldiv);
	Serial.printf("Frequency %f => PLL divior inte=%d, fraq=%d\n", f, intpart, frac);
	// set property group 0x40 index 0x00
	uint8_t buf[] = { 0x11, 0x40, 4, 0, (uint8_t)intpart, (uint8_t)(frac>>16), (uint8_t)(frac>>8), (uint8_t)frac };
	si4463_sendrecv(buf, 8, (uint8_t *)0, 0);
	return;
}

/*
 * channel: frequency is base + channel * step (here we use base 400 MHz, step 100 kHz for now)
 * condition: 0 to start immediatly
 * next state: 0: remain or better 8: RX (re-arm for new packet)
 * next states: next1 on preamble timeout, next2 on valid packet received, next3 on invalid packet received
 */
int si4463_startrx(uint8_t channel, uint8_t condition, uint16_t rxlen, uint8_t next1, uint8_t next2, uint8_t next3)
{
	uint8_t buf[8];
	buf[0] = 0x32;   // START_RX
	buf[1] = channel;
	buf[2] = condition;
	buf[3] = (uint8_t)(rxlen >> 8);
	buf[4] = (uint8_t)(rxlen);
	buf[5] = next1;
	buf[6] = next2;
	buf[7] = next3;
	si4463_sendrecv(buf, 8, (uint8_t *)0, 0);
	return 0;
}


int spi_receive(int waitcts, uint8_t *retbuf, int retlen)
{
	uint8_t buf[2];

	digitalWrite(HSPI_CS, LOW);
	if(waitcts) {
		while(1) {
			printf("Waiting for CTS");
			hspi->transfer(0x44);
			uint8_t cts= hspi->transfer(DUMMY);
			if(cts==0xFF) break;  // OK

			digitalWrite(HSPI_CS, HIGH);
			Serial.println("Waiting...\n");
			delay(50);  // should be shorter
			digitalWrite(HSPI_CS, LOW);
		}
	}
	if(retlen>0) {
		spi_getresponse(hspi, retbuf, retlen);
		Serial.printf("Response: (retlen=%d)\n", retlen);
		for(int i=0; i<retlen; i++) {
			Serial.printf("%02x ", retbuf[i]);
		}
	}
	digitalWrite(HSPI_CS, HIGH);
	Serial.println("---");
	return 0;
}



// "Public API" -> these do SPI transaction (the others assume to be within one?) 

/* wait for CTS (if waitcts is set), then read response */
int si4463_receive(int waitcts, uint8_t *retbuf, int retlen)
{
	Serial.printf("receive (%d, %d) called\n", waitcts, retlen);
	hspi->beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));
	int retval = spi_receive(waitcts, retbuf, retlen);
	hspi->endTransaction();
	return retval;
}

void si4463_waitcts() {
	Serial.println("waitcts called");
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

// This works for powering ip.
// Doing poweron() followed by power_up() does not. Need to check...
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
	   uint8_t rbuf[10];
	   spi_sendrecv(hspi, partinfo, 1, rbuf, 9);

	Serial.println("si4463 partinfo:");
	st_partinfo pi;
	res = si4463_partinfo(&pi);

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
	Serial.printf("Obtained partinfo: Chip %x, part %x, pbuild %x, id %x, customer %x, romid %x\n",
		pi->chiprev, pi->part, pi->pbuild, pi->id, pi->customer, pi->romid);
	return 0;
}

int si4463_getfifoinfo() {
	const uint8_t fifoinfo[] = { 0x15, 0x00 };
	uint8_t response[2];
	si4463_sendrecv(fifoinfo, 2, response, 2);
	return response[0];  // RX fifo, [1] is tx fifo (not used here)
}

// The readinfo command does not need to wait for CTS. The response is clocked out immediately after the command, do not toggle CS
int si4463_readfifo(uint8_t *buf, int len) {
	hspi->beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));
	digitalWrite(HSPI_CS, LOW);
	hspi->transfer(0x77);
	spi_getresponse(hspi, buf, len);
	digitalWrite(HSPI_CS, HIGH);
	hspi->endTransaction();
	return len;
}

int si4463_get_int_status() {
	return 0;
}

int si4463_starttx(uint8_t channel)
{
	uint8_t condition = 0;
	uint16_t txlen = 0;

	// Clear any interrupts
	si4463_get_int_status();

	uint8_t buf[8];
	buf[0] = 0x31;   // START_TX
	buf[1] = channel;
	buf[2] = condition;
	buf[3] = (uint8_t)(txlen >> 8);
	buf[4] = (uint8_t)(txlen);
	buf[5] = 0;
	buf[6] = 0;
	si4463_sendrecv(buf, 7, (uint8_t *)0, 0);
	return 0;
}

int si4463_getmodemstatus(uint8_t clearpend, st_modemstatus *status) {
	uint8_t buf[2];
	buf[0] = 0x22;  // GET_MODEM_STATUS
	buf[1] = clearpend;
	si4463_sendrecv(buf, 2, (uint8_t *)status, sizeof(st_modemstatus));
	status->afc_freq_offset = swaps(status->afc_freq_offset);
	return 0;
}

#define DATA_PIN 3
#define CLOCK_PIN 16
#define BUFFER_SIZE 128
uint8_t buffer[BUFFER_SIZE];
int bufferHead = 0, bufferTail = 0;

static void IRAM_ATTR onClockRisingEdge() {
	static int bit_count = 0;
	buffer[bufferHead] = (buffer[bufferHead] << 1) | digitalRead(DATA_PIN);
	bit_count++;
	if(bit_count >= 8) {
		bufferHead = (bufferHead + 1) % BUFFER_SIZE;
		bit_count = 0;
	}
}

// number of bytes available for reading in buffer
int si4463_getrawinfo() {
	return (bufferHead - bufferTail + BUFFER_SIZE) % BUFFER_SIZE;
}
int si4463_readraw(uint8_t *buf, int len) {
	for(int i=0; i<len; i++) {
		if( bufferHead == bufferTail)
			return -1;  // not enough data -- maybe return number of actually read bytes?
		buf[i] = buffer[bufferTail];
		bufferTail = (bufferTail + 1) % BUFFER_SIZE;
	}
	return len;
}


// returns byte if there is a new byte, -1 otherwise
int si4463_readdata() {
	if (bufferHead == bufferTail)
		return -1;
	uint8_t receivedByte = buffer[bufferTail];
	bufferTail = (bufferTail + 1) % BUFFER_SIZE;
	return receivedByte;
}


// activate IRQ handler for raw mode
void si4463_raw_activate() {
	// ESP CONFIG
	pinMode(CLOCK_PIN, INPUT);
	pinMode(DATA_PIN, INPUT);
	attachInterrupt(digitalPinToInterrupt(CLOCK_PIN), onClockRisingEdge, RISING);
	
	// Si4463 config  0x11(17): RX DATA CLK;   0x14(20) RX_DATA  // that is already in the default config
	uint8_t pincfg[] = { 0x13, 0x11, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00 };
        si4463_sendrecv( pincfg, 8, (uint8_t *)0, 0);
}



