# Test for Si4463 + ESP32S3

## Expected wiring (change in Test/src/si4463.cpp):
| Si4463 pin | ESP32 pin |
|------------|-----------|
| nSEL       | IO10/SPI_CS
| SCK        | IO12/SPI_CLK
| MISO       | IO13/SPI_Q
| MOSI       | IO11/SPI_D
| SDN        | IO01
| IRQ        | IO02
| GPIO_0     | IO16 (direct mode/clk)
| GPIO_1     | IO03 (direct mode/data)

## Compile&upload with platformio
- pio run --target=upload

## Monitor
- pio device monitor -b 115200

## On monitor
| Command   | Function |
|---|---|
| P         | -> Powercycle |
| Fnnn.nn   | -> Set RX Frequency |
| C         | -> Send WDS config to Si4463 |
| R         | -> Run (START_RX) |
| S         | -> Stop (+print statistics) |
| Vxx       | -> Verbosity (bitmask, hex) x=00..ff, vC color on, vc color off |
