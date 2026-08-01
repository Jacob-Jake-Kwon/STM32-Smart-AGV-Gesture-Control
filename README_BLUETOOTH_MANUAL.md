# Auto / Manual / Stop Car — HC-05 or HC-06

## Important pin decision

USART1 cannot be used because PA9 and PA10 are already connected to the L298N
direction inputs. Bluetooth therefore uses USART2:

| HC-05 / HC-06 | STM32F411RE |
|---|---|
| TXD | PA3 / USART2_RX |
| RXD | PA2 / USART2_TX |
| VCC | 5 V on common breakout boards |
| GND | GND |

The bare Bluetooth module itself is 3.3 V. Use the voltage stated for your
specific breakout board.

Default serial rate in this project: 9600 baud.

## Blue-button modes

- Power on: STOP — LED off
- First press: AUTO — LED continuously on
- Second press: MANUAL — LED blinking
- Third press: STOP — LED off
- Repeat

## Bluetooth protocol

Send newline-terminated commands:

- `F` forward
- `B` backward
- `L` rotate left
- `R` rotate right
- `FL` forward-left
- `FR` forward-right
- `BL` backward-left
- `BR` backward-right
- `S` stop

Example bytes:

`F\n`

The ESP32 glove should send a command repeatedly, around every 100–200 ms.
The STM32 stops the motors if no valid command is received for 600 ms.

## Safety

The front ultrasonic sensor remains active in MANUAL mode. Forward and
forward-diagonal commands are blocked when the front obstacle is too close.
Backward and in-place turn commands remain available to escape.

## Build

Import as an existing STM32CubeIDE project, clean the project, and build.
Do not regenerate the IOC unless you intend to preserve the custom task and
application files.
