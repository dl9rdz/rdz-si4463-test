typedef struct __attribute__((__packed__)) si4463_partinfo {
//	uint8_t cts;
	uint8_t chiprev;
	uint16_t part;
	uint8_t pbuild;
	uint16_t id;
	uint8_t customer;
	uint8_t romid;
} st_partinfo;

void si4463_init();

void si4463_test();

void si4463_reset();
int si4463_configure();
int si4463_partinfo(st_partinfo *pi);
