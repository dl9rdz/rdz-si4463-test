all:
	pio run --target=upload
	pio device monitor

mon:
	pio device monitor
