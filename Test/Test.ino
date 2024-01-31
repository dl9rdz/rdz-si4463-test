#include <ctype.h>
#include "src/si4463.h"
#include "src/logger.h"

enum RxResult { RX_OK, RX_TIMEOUT, RX_ERROR, RX_UNKNOWN, RX_NOPOS };

int running = 1;
float freq = 400000000;  // Freq in Hz.

typedef struct st_rxstat {
	uint32_t start;
	uint16_t framecnt;
	uint16_t manerr;
	uint16_t syncerr;
	uint16_t hamcor;
	uint16_t hamerr;
} t_rxstat;
t_rxstat rxstat;

void setup() {
    Serial.begin(115200);
    logSetMask(LOG_INFO | LOG_RADIO | LOG_RXFRM | LOG_RXTLM | LOG_RXRAW | LOG_RXSTAT );
    delay(4000);     // Just to get all log messages on the serial port :)
    si4463_init();   // Initialize SPI bus
    si4463_reset();  // Power-cycle the radio chip
    // si4463_poweron();
    si4463_configure();   // Send all configuration stuff from  Wireless Development Suite
}

#define DFM_FRAMELEN 33

void printTimestamp(uint16_t log_level, uint32_t *last) {
	uint32_t t = millis();

	uint32_t ms = t;
	uint32_t s = t / 1000; ms -= 1000 * s;
	uint32_t m = s / 60; s -= 60 * m;
	uint32_t h = m / 60; m -= 60 * h;
	h = h % 24;
	logPrint(log_level, "%02d:%02d:%02d.%03d ", h, m, s, ms);
	if(last) {
		uint32_t diff = t - *last;
		*last = t;
		logPrint(log_level, "(+%d.%03d): ", diff/1000, (diff-diff/1000*1000));
	}
}

void printRaw(const char *label, int len, int ret, const uint8_t *data)
{
	//logPrint( LOG_RXFRM, "%s(%d): ", label, ret);
	logPrint( LOG_RXFRM, "%s:", label);
        int i;
        for(i=0; i<len/2; i++) {
                logPrint( LOG_RXFRM, "%02X", data[i]);
        }
	logPrint( LOG_RXFRM, "%01X ", data[i]&0x0F);
}



void decodeDAT(uint8_t *dat)
{
        logPrint( LOG_RXTLM, " DAT[%d]: ", dat[6]);

        // We handle this case already here, because we need to update dfmstate.datesec before the cycle complete handling 
        if( dat[6]==8 ) {
                int y = (dat[0]<<4) + (dat[1]>>4);
                int m = dat[1]&0x0F;
                int d = dat[2]>>3;
                int h = ((dat[2]&0x07)<<2) + (dat[3]>>6);
                int mi = (dat[3]&0x3F);
                logPrint( LOG_RXTLM, "Date: %04d-%02d-%02d %02d:%02dz ", y, m, d, h, mi);
        }
        else if( dat[6]>8 ) return; // we ignore those...

        switch(dat[6]) {
        case 0:
                logPrint( LOG_RXTLM, "Packet counter: %d", dat[3]); 
                break;
        case 1:
                {
                int val = (((uint16_t)dat[4])<<8) + (uint16_t)dat[5];
                logPrint( LOG_RXTLM, "UTC-msec: %d", val);
                }
                break;
        case 2:
                {
                float lat, vh;
                lat = (int32_t)(((uint32_t)dat[0]<<24) + ((uint32_t)dat[1]<<16) + ((uint32_t)dat[2]<<8) + ((uint32_t)dat[3]));
                vh = ((uint16_t)dat[4]<<8) + dat[5];
		logPrint( LOG_RXTLM, "GPS-lat: %f, v_h: %f", lat*0.0000001, vh*0.01 );
                }
                break;
        case 3:
                {
                float lon, dir;
                lon = (int32_t)(((uint32_t)dat[0]<<24) + ((uint32_t)dat[1]<<16) + ((uint32_t)dat[2]<<8) + (uint32_t)dat[3]);
                dir = ((uint16_t)dat[4]<<8) + dat[5];
                lon = lon*0.0000001;
		logPrint( LOG_RXTLM, "GPS-lon: %f, dir: %d", lon, dir);
                }
                break;
        case 4:
                {
                float alt, vv;
                alt = ((uint32_t)dat[0]<<24) + ((uint32_t)dat[1]<<16) + ((uint32_t)dat[2]<<8) + dat[3];
                vv = (int16_t)( ((int16_t)dat[4]<<8) | dat[5] );
		logPrint( LOG_RXTLM, "GPS-hgt: %f, vv:%f", alt*0.01, vv*0.01);
                }
                break;
        case 8:
                // handled above
                break;
        default:
                logPrint( LOG_RXTLM, "(?)" );
                break;
        }
}


void decodeCFG(uint8_t *cfg)
{
}


#define B 8
#define S 4
        uint8_t hamming_conf[ 7*B];  //  7*8=56
        uint8_t hamming_dat1[13*B];  // 13*8=104
        uint8_t hamming_dat2[13*B];

        uint8_t block_conf[ 7*S];  //  7*4=28
        uint8_t block_dat1[13*S];  // 13*4=52
        uint8_t block_dat2[13*S];


// Error correction for hamming code
        uint8_t H[4][8] =  // extended Hamming(8,4) particy check matrix
             {{ 0, 1, 1, 1, 1, 0, 0, 0},
              { 1, 0, 1, 1, 0, 1, 0, 0},
              { 1, 1, 0, 1, 0, 0, 1, 0},
              { 1, 1, 1, 0, 0, 0, 0, 1}};
        uint8_t He[8] = { 0x7, 0xB, 0xD, 0xE, 0x8, 0x4, 0x2, 0x1};  // Spalten von H:
                                                            // 1-bit-error-Syndrome


        
uint32_t bits2val(const uint8_t *bits, int len) {
        uint32_t val = 0;
        for (int j = 0; j < len; j++) {
                val |= (bits[j] << (len-1-j));
        }
        return val;
}



// returns 0: ok   >0: 1 error was corrected -1: uncorrectable error
int check(uint8_t code[8]) {
        int i, j;            
        uint32_t synval = 0;  
        uint8_t syndrom[4];   
        int ret=0;

        for (i = 0; i < 4; i++) { 
                syndrom[i] = 0;
                for (j = 0; j < 8; j++) { 
                        syndrom[i] ^= H[i][j] & code[j];
                }
        }
        synval = bits2val(syndrom, 4);
        if (synval) {
                ret = -1;
                for (j = 0; j < 8; j++) {   // 1-bit-error
                        if (synval == He[j]) {  // reicht auf databits zu pruefen, d.h.
                                ret = j+1;          // (systematischer Code) He[0..3]
                                break;
                        }
                }
        }
        else ret = 0;
        if (ret > 0) code[ret-1] ^= 0x1;

        return ret;
}

static int use_ecc = 1;

// Extended (8,4) Hamming code
// Return number of corrected bits, -1 if uncorrectable error
int hamming(uint8_t *ham, int L, uint8_t *sym) {
        int i, j;
        int ret = 0;               // DFM: length L = 7 or 13
        for (i = 0; i < L; i++) {  // L bytes (4bit data, 4bit parity)
                if (use_ecc) {
                        int res = check(ham+8*i);
                        if( res<0 ) ret = -1;
                        else if ( ret >= 0 && res > 0 ) ret++;
                }
                // systematic Hamming code: copy bits 0..3
                for (j = 0; j < 4; j++) {
                        sym[4*i+j] = ham[8*i+j];
                }
        }
        return ret;
}




#define bitpick(value,bitpos) (((value)>>(7-(bitpos)))&0x01)
// Input: str: packed data, MSB first
void deinterleave(uint8_t *str, int L, uint8_t *block) {
        int i, j;
        for (j = 0; j < B; j++) {  // L = 7 (CFG), 13 (DAT1, DAT2)
                for (i = 0; i < L; i++) {
                        block[B*i+j] = bitpick( str[(L*j+i)/8], (L*j+i)&7 )?0:1;
                }
        }
}

void bitsToBytes(uint8_t *bits, uint8_t *bytes, int len)
{
        int i;
        for(i=0; i<len*4; i++) {
                bytes[i/8] = (bytes[i/8]<<1) | (bits[i]?1:0);
        }
        bytes[(i-1)/8] &= 0x0F;
}

int decodeFrameDFM(uint8_t *data) {
        deinterleave(data, 7, hamming_conf);
        deinterleave(data+7, 13, hamming_dat1);
        deinterleave(data+20, 13, hamming_dat2);
  
        int ret0 = hamming(hamming_conf,  7, block_conf);
        int ret1 = hamming(hamming_dat1, 13, block_dat1);
        int ret2 = hamming(hamming_dat2, 13, block_dat2);
        //Serial.printf("Hamming returns %d %d %d -- %d\n", ret0, ret1, ret2, ret0|ret1|ret2);
	if(ret0<0) rxstat.hamerr++; else rxstat.hamcor += ret0;
	if(ret1<0) rxstat.hamerr++; else rxstat.hamcor += ret1;
	if(ret2<0) rxstat.hamerr++; else rxstat.hamcor += ret2;

        byte byte_conf[4], byte_dat1[7], byte_dat2[7];
        bitsToBytes(block_conf, byte_conf, 7);
        bitsToBytes(block_dat1, byte_dat1, 13);
        bitsToBytes(block_dat2, byte_dat2, 13);

	static uint32_t lastFrm = 0;
	printTimestamp( LOG_RXFRM, &lastFrm );
	logPrint( LOG_RXFRM, "HAM(%d/%d/%d) ", ret0, ret1, ret2);
        printRaw("CFG", 7, ret0, byte_conf);
        printRaw("DAT", 13, ret1, byte_dat1);
        printRaw("DAT", 13, ret2, byte_dat2);
	if(!logEnabled(LOG_RXTLM)) logPrint( LOG_RXFRM, "\n" );
        if (ret0>=0) decodeCFG(byte_conf);
        if (ret1>=0 && ret1<=4) decodeDAT(byte_dat1);
        if (ret2>=0 && ret2<=4) decodeDAT(byte_dat2);
        logPrint( LOG_RXTLM, "\n");
        // Consistent with autorx: If more than 4 corrected bit errors in DAT block, assume it is possibly corrupt and
        // don't treat it as a correct frame (ttgo display shows data anyway, but it is not sent to external sites)
        if(ret1>4 || ret2>4) return RX_ERROR;
        return (ret0|ret1|ret2)>=0 ? RX_OK : RX_ERROR;
}

int hammingDistance(uint32_t value1, uint32_t value2) {
    // XOR the two values
    uint32_t xorResult = value1 ^ value2;

    // Use __builtin_popcount for hardware-specific population count
    int differenceCount = __builtin_popcount(xorResult);

    return differenceCount;
}

int procbyte(uint8_t dt) {
        static uint8_t data[1024];
        static uint32_t rxdata = 0;
        static uint8_t rxbitc = 0;
        static uint8_t rxbyte = 0;
        static uint8_t rxsearching = 1;
        static uint8_t rxp;
        static int rssi=0, fei=0, afc=0;
        static uint8_t invers = 0;
	static uint8_t bad_manchester = 0; 
        
        for(int i=0; i<8; i++) {
                uint8_t d = (dt&0x80)?1:0;
                dt <<= 1;
                rxdata = (rxdata<<1) | d;
                if( (rxbitc&1)==0 ) {
                        // "bit1"
                        rxbyte = (rxbyte<<1) | d;
                } else {
                        // "bit2" ==> 01 or 10 => 1, otherweise => 0
                        // not here: (10=>1, 01=>0)!!! rxbyte = rxbyte ^ d;
			if( (rxdata&3)==0 || (rxdata&3)==3 ) {  // bad manchaster 
				bad_manchester++;
			}
                }
                //
                if(rxsearching) {
                        int d1 = hammingDistance(rxdata, 0x6566A5AA);
			int d2 = hammingDistance(rxdata, 0x9A995A55);
			if(d1 <=4 || d2 <= 4) {
				int d = d2;
				if( d1<d ) {
					invers = 1;
					d = d1;
				}
				static uint32_t last=0;
				if(logEnabled(LOG_RXRAW)) printTimestamp(LOG_RXRAW, &last);
				if(d==0)
					logPrint(LOG_RXRAW, "SYNC[%08x/--] ", rxdata);
				else
					logPrint(LOG_RXRAW, "SYNC[%08x/%de] ", rxdata, d);
				rxstat.syncerr += d;
                                rxsearching = false;
                                rxbitc = 0;
                                rxp = 0;
                                rxbyte = 0;
				bad_manchester = 0;
                        }
                } else {
                        rxbitc = (rxbitc+1)%16; // 16;
                        if(rxbitc == 0) { // got 8 data bit
                                if(invers) rxbyte=~rxbyte;
                                data[rxp++] = rxbyte&0xff; // (rxbyte>>1)&0xff;
                                if(rxp>=DFM_FRAMELEN) {
                                        rxsearching = true;
					rxdata = 0;
					if( logEnabled(LOG_RXRAW) ) {
						logPrint(LOG_RXRAW, "Data: ");
                                        	for(int i=0; i<DFM_FRAMELEN; i++) {
							logPrint(LOG_RXRAW, "%02x ", data[i]); 
						}
						if(bad_manchester>0)
							logPrint(LOG_RXRAW, " [%d inval. Manch.]\n", bad_manchester);
						else
							logPrint(LOG_RXRAW, "\n");
					}
                                        decodeFrameDFM(data);
					rxstat.framecnt++;
					rxstat.manerr += bad_manchester;
                                        //haveNewFrame = 1;
                                }
                        }
                }
        }
        return 0;
}

void cmd_power(const char *cmd)
{
	if(cmd[1]=='0') {
		Serial.println("Shutdown [SDN=1]");
		si4463_poweroff();
		running = 0;
		return;
	}
	if(cmd[1]=='\r'||cmd[1]=='\n'||cmd[1]==0 ) {
		Serial.println("Shutdown for powercycle [SDN=1]");
		si4463_poweroff();
		delay(1);  // 1ms delay. spec requires >10us, so this is generous
		running = 0;
	}
	if(cmd[1]=='\r'||cmd[1]=='\n' || cmd[1]==0 || cmd[1]=='1') {
		Serial.println("Powerup [SDN=0], patches + RF_POWER_UP SPI command");
		si4463_poweron();
		//si4463_test();
		//si4463_power_up_cmd();
		running = 0;

		st_partinfo pi;
		si4463_partinfo(&pi);
    		Serial.printf("Partinfo: Chip %x, part %x, pbuild %x, id %x, customer %x, romid %x\n",
       			pi.chiprev, pi.part, pi.pbuild, pi.id, pi.customer, pi.romid);

	}
}

extern uint8_t Radio_Configuration_Data_Array[];

void cmd_config(const char *config)
{
	// TODO: Support multiple configs
	si4463_send_config(Radio_Configuration_Data_Array);

	// Config contains WDS-generated default freq, switch to "right" freq
	Serial.printf("Config sent, setting freq to %f\n", freq);
	si4463_setfreq(freq);
}

void cmd_freq(const char *freqstr)
{
	int val = atoi(freqstr+1);
	if (val < 100) {
		freq = 400000000 + val * 100000;
	} else if (val > 100000) {
		// frequency in kHz
		freq = 1000.0 * val;
	} else {
		// frequency in MHz
		freq = atof(freqstr+1) * 1000000;
	}
	Serial.printf("(val:%d) Setting frequency to %f MHz\n", val, 0.000001 * freq);
	si4463_setfreq(freq);
}


void cmd_run(const char *run) {
	// Start receiver
	// use freq is set on channel => need channel 0
	si4463_startrx(0, 0, /*255*/ /*8191*/ /*66*/ /*32*8*2*/ 0 /*len*/, 0, 0, 0);
	running = 1;
	rxstat = {0};
	rxstat.start = millis();
}

void cmd_stop() {
	uint32_t stop = millis();
	uint32_t diff = stop - rxstat.start;
	logPrint( LOG_INFO, "Stop t(run)=%d.%ds RX: %d frames, %d sync biterr, %d bad machester Hamming: %d corrected, %d fail\n", diff/1000, diff-(diff/1000*1000), rxstat.framecnt,
		rxstat.syncerr, rxstat.manerr, rxstat.hamcor, rxstat.hamerr );	
	// TODO: print some statistics
	// TODO stop the RX
	running = 0;
}

void cmd_verb(const char *verb) {
	int val = atoi(verb+1);
	if(val || verb[1]=='0') {
		logSetMask(val);
		return;
	} else if (verb[1]=='c') {
		logSetColor(0);
	} else if (verb[1]=='C') {
		logSetColor(1);
	}
}

void cmd_info(const char *info) {
	info++;
	while(*info) {
		switch(tolower(*info)) {
		case 'p':
		{
			st_partinfo pi;
			si4463_partinfo(&pi);
    			Serial.printf("Partinfo: Chip %x, part %x, pbuild %x, id %x, customer %x, romid %x\n",
        			pi.chiprev, pi.part, pi.pbuild, pi.id, pi.customer, pi.romid);
			break;
		}
		case 's':
		{
			st_modemstatus status;
			si4463_getmodemstatus(0, &status);
			Serial.printf("Modemstatus: Pend 0x%02x Status 0x%02x RSSI: Curr %d Latch %d Ant1 %d Ant2 %d AFC Offset: %d\n",
				status.modem_pend, status.modem_status, status.curr_rssi, status.latch_rssi,
				status.ant1_rssi, status.ant2_rssi, status.afc_freq_offset);
			break;
		}
		}
		info++;
	}
}


void handle_cmd(char *str) {
	switch(tolower(*str)) {
	case 'p':
		cmd_power(str);
		break;
	case 'c':
		cmd_config(str);
		break;
	case 'f':
		cmd_freq(str);
		break;
	case 'r':
		cmd_run(str);
		break;
	case 'i':
		cmd_info(str);
		break;
	case 'v':
		cmd_verb(str);
		break;
	case 's': 
		cmd_stop();
		break;
	}
}

static char serin[255];
static int n_serin = 0;
void handle_serial() {
	while(Serial.available()) {
		char ch = Serial.read();
		if(ch=='\n' || n_serin>=254) {
			handle_cmd(serin);
			n_serin = 0;
		} else {
			serin[n_serin++] = ch;
		}
	}
}

void loop() {
    static int blubb = 0;
    st_partinfo pi;
    int res = si4463_partinfo(&pi);
    logPrint(LOG_INFO, "Partinfo: Chip %x, part %x, pbuild %x, id %x, customer %x, romid %x\n",
        pi.chiprev, pi.part, pi.pbuild, pi.id, pi.customer, pi.romid);

    // adjust offset
    // in config. MODEM_CLKGEN_BAND (0x51) is 0x09, ie. SY_SEL set (Npresc=2) ad BAND 1 (outdiv=6)
#define outdiv 10
#define Npresc 2
    //float offset = (1<<19) * outdiv * 100000.0 / Npresc / 26000000;
    //printf("Using offset value %f\n", offset);
    //si4463_setfreqoffset((uint16_t)(int)3028);//-offset);
     // in real, 10082 == 100 kHz, so 3528 is 33 khz

    // receive immediately on ch35 (403.5 MHz)
    //si4463_startrx(33, 0, /*255*/ /*8191*/ /*66*/ 0 /*len*/, 0, 0, 0);
    si4463_startrx(35, 0, /*255*/ /*8191*/ /*66*/ /*32*8*2*/ 0 /*len*/, 8, 8, 8);
    int last = millis();
    si4463_raw_activate();
    while(1) {
        //delay(1000);
	handle_serial();
        delay(10);
	if(!running) continue;
        //int n = si4463_getfifoinfo();
	int nraw = si4463_getrawinfo();
        if(nraw==0) {
            if( ((++blubb)&15) ==0) Serial.println("Empty fifo");
            delay(50);
    	    //si4463_startrx(35, 0, /*255*/ /*8191*/ /*66*/ 32*8*2 /*len*/, 8, 8, 8);
            continue;
        }
        //uint8_t buf[n];
        uint8_t bufraw[nraw];
        //si4463_readfifo(buf, n);
	si4463_readraw(bufraw, nraw);
#if 0
        for(int i=0; i<n; i++) {
            logPrint( LOG_RXDBG, "%02x ", buf[i]);
            //procbyte(buf[i]);
        }
#endif
        for(int i=0; i<nraw; i++) {
            logPrint( LOG_RXDBG, "%02x ", bufraw[i]);
            procbyte(bufraw[i]);
        }
        // reset packet length, so RX will continue
        //si4463_startrx(35, 0, 66/*len*/, 0, 0, 0);

        if( (millis()-last)>1000 ) {
            last = millis();
            st_modemstatus status;
            si4463_getmodemstatus(0, &status);
            logPrint( LOG_RXSTAT, "RSSI: %d   AFC: %d\n", status.curr_rssi, status.afc_freq_offset);
        }
    }
    delay(20000);
}
