typedef struct __attribute__((__packed__)) si4463_partinfo {
//	uint8_t cts;
	uint8_t chiprev;
	uint16_t part;
	uint8_t pbuild;
	uint16_t id;
	uint8_t customer;
	uint8_t romid;
} st_partinfo;

typedef struct __attribute__((__packed__)) si4463_modemstatus {
	uint8_t modem_pend;
	uint8_t modem_status;
	uint8_t curr_rssi;
	uint8_t latch_rssi;
	uint8_t ant1_rssi;
	uint8_t ant2_rssi;
	uint16_t afc_freq_offset;
} st_modemstatus;

void si4463_init();

void si4463_test();

void si4463_poweron();
void si4463_poweroff();
void si4463_reset();

int si4463_send_config(uint8_t *initdata);
int si4463_configure();

void si4463_setfreq(float f);
int si4463_setfreqoffset(uint16_t offset);


int si4463_partinfo(st_partinfo *pi);
int si4463_getmodemstatus(uint8_t clear, st_modemstatus *status);

int si4463_startrx(uint8_t channel, uint8_t condition, uint16_t rxlen, uint8_t next1, uint8_t next2, uint8_t next3);

int si4463_getfifoinfo();
int si4463_readfifo(uint8_t *buf, int len);

// Maybe make similar to fifo, i.e. getirqfifo, readirqfifo?
void si4463_raw_activate();
int si4463_readdata();

int si4463_getrawinfo();
int si4463_readraw(uint8_t *buf, int len);

