all:
	pio run --target=upload
	pio device monitor -b 115200

mon:
	pio device monitor -b 115200
