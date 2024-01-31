#include <Arduino.h>
#include <SPI.h>
// Just

#include "logger.h"
#include "si4463.h"
#include "radio_config_Si4463.h"
//#include "radio_config_Si4463_dfm.h"
//#include "radio_config_Si4463_rs41.h"

// SPI Interface
#define HSPI_SCLK 12
#define HSPI_MISO 13
#define HSPI_MOSI 11
#define HSPI_CS 10

// Shutdown pin of Si4463
#define SDN 1

// Clock and data for Si4463
// (we assume CLK is on Si4463's GPIO0 and data on GPIO1 -- hardcoded in GPIO_PIN_CFG register)
#define DATA_PIN 3
#define CLOCK_PIN 16


static uint8_t buf[] = { 0x13, 0x00, 0x00, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t power[] = { 0x02, 0x01, 0x00, 0x01, 0xc9, 0xc3, 0x80  };
static uint8_t partinfo[] = { 0x01 };

uint8_t Radio_Configuration_Data_Array[] = RADIO_CONFIGURATION_DATA_ARRAY;
uint8_t Radio_Patches_Powerup_Array[] = RADIO_PATCHES_POWERUP_ARRAY;


// Initialized by si4463_init()
static SPIClass *hspi;


/******************************************************************************
 * Low-level SPI functions and initialization
 * These function assume that the caller locks and unlocks the SPI bus
 * (beginTransaction / endTransaction
 ******************************************************************************/

// Send a SPI message of some length
static void spi_sendcmd(const uint8_t *buf, int len) {
	for(int i=0; i<len; i++) {
		hspi->transfer(buf[i]);
	}
}

// Read a SPI response of some length
#define DUMMY 0xFF
static void spi_getresponse(uint8_t *buf, int len) {
	for(int i=0; i<len; i++) {
		buf[i] = hspi->transfer(DUMMY);
	}
}

// Read a SPI response of some length, waitign for CTS if waitcts is true
static int spi_receive(int waitcts, uint8_t *retbuf, int retlen)
{
	if(waitcts==0 && retlen==0)
		return 0;

	digitalWrite(HSPI_CS, LOW);
	if(waitcts) {
		// TODO: Add timeout
		while(1) {
			logPrint(LOG_SPI, "spi_receive: Checking for CTS\n");
			hspi->transfer(0x44);
			uint8_t cts = hspi->transfer(DUMMY);
			// If we don't get CTS (0xFF) back, we need to release CS and try again after some time
			// If we get CTS, we have to keep CS active (low) and read the data
			if(cts==0xFF) break;  // OK

			digitalWrite(HSPI_CS, HIGH);
			// TODO iteration count & timeout
			delay(1);
			digitalWrite(HSPI_CS, LOW);
		}
	}
	if(retlen>0) {
		spi_getresponse(retbuf, retlen);
		logPrint(LOG_SPI, "spi_receive: Response(len=%d): ", retlen);
		for(int i=0; i<retlen; i++) {
			logPrint(LOG_SPI, "%02x ", retbuf[0]);
		}
	}
	digitalWrite(HSPI_CS, HIGH);
	logPrint(LOG_SPI, "---\n");
	return 0;
}

// Initialize library 
void si4463_init() {
	logPrint(LOG_RADIO, "si4463_init: Initializing SPI bus\n");
	hspi = new SPIClass(HSPI);
	hspi->begin(HSPI_SCLK, HSPI_MISO, HSPI_MOSI, -1); //SCLK, MISO, MOSI, SS

	digitalWrite(HSPI_CS, HIGH);
	digitalWrite(SDN, LOW);

	pinMode(HSPI_CS, OUTPUT);  // CnS
	pinMode(SDN, OUTPUT);  // CnS
}

/******************************************************************************
 * Internal SPI functions for Si4463
 * These lock the SPI bus (beginTransaction) before calling any SPI function above
 ******************************************************************************/

// TODO: This still needs some cleanup...

/* wait for CTS (if waitcts is set), then read response */
int si4463_receive(int waitcts, uint8_t *retbuf, int retlen)
{
	logPrint(LOG_SPI, "si4463_receive (waitcts: %d, retlen: %d)\n", waitcts, retlen);
	hspi->beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));
	int retval = spi_receive(waitcts, retbuf, retlen);
	hspi->endTransaction();
	return retval;
}


void si4463_waitcts() {
	logPrint(LOG_RADIO, "si4463_waitcts()\n");
	hspi->beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));
	int retval = spi_receive(1, (uint8_t *)0, 0);
	hspi->endTransaction();
	//si4463_receive(1, (uint8_t*)0, 0);
}	

// This is not used???
int si4463_sendcommand(uint8_t *cmd, int cmdlen)
{
	hspi->beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));
	digitalWrite(HSPI_CS, LOW);
	spi_sendcmd(cmd, cmdlen);
	digitalWrite(HSPI_CS, HIGH);
	hspi->endTransaction();
	return 0;
}


void si4463_sendrecv(const uint8_t *cmd, int cmdlen, uint8_t *resp, int resplen) {
	uint8_t ctsval;
	hspi->beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));

#if 0
	// let's not use that?
	// usually sendrecv waits until CTS a the end anyway, so we could also manually wait for CTS?
	// unless we have some case for waiting?
	// But this code is bad, as it does not pull CS HIGH before sending the next command.
	// If really needed we could ccall waitcts()
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
#endif

	// Send command
	logPrint(LOG_SPI, "Sending: ");
	for(int i=0; i<cmdlen; i++) { logPrint(LOG_SPI, "%02x ", cmd[i]); }
	digitalWrite(HSPI_CS, LOW);
	spi_sendcmd(cmd, cmdlen);
	digitalWrite(HSPI_CS, HIGH);

	// min 80 ns
	delay(1); 
	digitalWrite(HSPI_CS, LOW);
	while(1) { // wait until CTS
		hspi->transfer(0x44);   // read CMD buffer
		ctsval = hspi->transfer(DUMMY);
		if(ctsval==0xFF) { 
			logPrint(LOG_SPI, " [CTS OK] ");
			if(resplen>0) {
				logPrint(LOG_SPI, "Response (%d bytes): ", resplen);
				spi_getresponse(resp, resplen);
				for(int i=0; i<resplen; i++) { logPrint(LOG_SPI, "%02X ", resp[i]); }
				logPrint( LOG_SPI, "\n");
			}
			digitalWrite(HSPI_CS, HIGH);
			break;
		}
		digitalWrite(HSPI_CS, HIGH);
		// TODO: Maybe add timeout and error
		delay(100);
		logPrint( LOG_SPI, "CTSWAIT\n");
	}
	if(ctsval==0xff) {  // should always be the case!
		//ctsOK = 1;
	}
	hspi->endTransaction();
}


// Send patches (created using WDS) and RF_POWER_UP command
// Timing is important, so let's do this automatically as part of poweron()
int si4463_power_up_cmd() {
	logPrint( LOG_RADIO, "si4463_power_up_cmd()\n" );
	uint8_t *initdata = Radio_Patches_Powerup_Array;
	return si4463_send_config(initdata);
}

  
/******************************************************************************
 * Public API functions
 ******************************************************************************/
// Important note on powerup sequence:
// https://community.silabs.com/s/article/si4x6x-c2a-si4x55-c2a-startup-sequence
// First SPI transaction after SDN low has to finish in less than 14ms.
// (The waitcts is a SPI transaction...)
// see also https://www.silabs.com/documents/public/application-notes/AN633.pdf
// (AN633 Rev 0.9)

#define USEPATCHES 1

void si4463_poweron() {
	logPrint( LOG_RADIO, "si4463_poweron()\n" );
	digitalWrite(SDN, LOW);

	// AN633v0.9, p.25: After pulling SDN low, POR takes a maximum of 6ms
	// same page, a few lines below, and also next page: Wait 14 ms
	delay(14);

	// AN633v0.9: Then make sure to send a SPI transaction in less than 4ms
	// counted from falling NSEL to rising NSEL edge.
	// Could do a power command directly, but lets do a CTS poll for now.
	si4463_waitcts();

	uint8_t res[40];
	// So let's send the power on command
#if USEPATCHES
	si4463_power_up_cmd();
#else
	si4463_sendrecv(power, sizeof(power), res, 0);
#endif


	// So let's wait for CTS...
	si4463_waitcts();

	//Serial.printf("Power ok: %x\n", res[0]);
	//delay(2200);
}

void si4463_poweroff() {
	logPrint( LOG_RADIO, "si4463_poweroff()\n" );
	digitalWrite(SDN, HIGH);
}


void si4463_reset() {
	si4463_poweroff();
	delay(1);
	si4463_poweron();
#if 0
	Serial.println("Si4463 power off/on reset");
	digitalWrite(SDN, HIGH);
	delay(500);
	digitalWrite(SDN, LOW);
	delay(500);
	ctsOK = 0;  // need to wait for CTS
	delay(100);
	si4463_waitcts();
#endif
}



/*******************************************************************************
 * Configuration commands
 */

int si4463_send_config(uint8_t *initdata) {
	int count = 0;
	logPrint( LOG_RADIO, "si4463_send_config(%p)...\n", initdata);
	uint8_t rbuf[20];
	while (*initdata != 0) {
		uint8_t numbytes = *initdata++;
		if(numbytes>16) {
			logPrint( LOG_RADIO, "FATAL ERROR: Number of bytes in init sequence exceeds 16: %d\n", numbytes);
			return -1;
		}
		si4463_sendrecv(initdata, numbytes, rbuf, 0);
		initdata += numbytes;
		count++;
	}
	logPrint( LOG_RADIO, "...%d commands sent\n", count);
	return 0;
}

// Send all configuration commands from Radio_Configuration_Data_Array
// (created using Wireless Development Suite)
int si4463_configure() {
	uint8_t *initdata = Radio_Configuration_Data_Array;
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
	logPrint( LOG_RADIO, "si4463_setfreq: Frequency %f => PLL divior inte=%d, fraq=%d\n", f, intpart, frac);
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
	logPrint( LOG_RADIO, "si4463_startrx()\n" );
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


static unsigned short swaps( unsigned short val)
{
	return ((val & 0xff) << 8) | ((val & 0xff00) >> 8);
}

int si4463_partinfo(st_partinfo *pi) {
	si4463_sendrecv(partinfo, 1, (uint8_t *)pi, 9);
	pi->part = swaps(pi->part);
	pi->id = swaps(pi->id);
	logPrint( LOG_RADIO, "si4463_partinfo(): Chip %x, part %x, pbuild %x, id %x, customer %x, romid %x\n",
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
	spi_getresponse(buf, len);
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
	logPrint( LOG_RADIO, "si4463_raw_activate(): Setting up IRQ handler for RAW RX mode\n");
	// ESP CONFIG
	pinMode(CLOCK_PIN, INPUT);
	pinMode(DATA_PIN, INPUT);
	attachInterrupt(digitalPinToInterrupt(CLOCK_PIN), onClockRisingEdge, RISING);
	
	// Si4463 config  0x11(17): RX DATA CLK;   0x14(20) RX_DATA  // that is already in the default config
	uint8_t pincfg[] = { 0x13, 0x11, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00 };
        si4463_sendrecv( pincfg, 8, (uint8_t *)0, 0);
}



