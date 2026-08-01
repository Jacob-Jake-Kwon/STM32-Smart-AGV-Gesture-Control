# STM32F411RE FreeRTOS Maze Car

This project uses the working L298N motor-control base and adds three ultrasonic
sensors, FreeRTOS tasks, local maze navigation, and blue-button start/stop.

## L298N wiring

| L298N | STM32F411RE |
|---|---|
| ENA | PA8 / TIM1_CH1 |
| IN1 | PA9 |
| IN2 | PA10 |
| ENB | PB4 / TIM3_CH1 |
| IN3 | PB5 |
| IN4 | PB6 |
| GND | STM32 GND |

Remove the ENA and ENB jumper caps when PWM pins are connected.

## Ultrasonic wiring

| Sensor | TRIG | ECHO |
|---|---|---|
| Front | PB0 | PB10 |
| Left | PB1 | PB12 |
| Right | PB2 | PB13 |

Typical HC-SR04 ECHO outputs are 5 V. Use a voltage divider on every ECHO line
before connecting it to the STM32.

Suggested sensor direction:

- Front: straight ahead
- Left: approximately 35–45 degrees left
- Right: approximately 35–45 degrees right

## Blue button

The Nucleo blue USER button on PC13 toggles:

- First press: start automatic driving
- Second press: stop immediately
- Further presses repeat start/stop

The green status LED on PA5 is on while automatic driving is active.

## Navigation behavior

- Drives forward while the front is clear.
- Corrects away from close side walls.
- At a wide junction, it may select the side with substantially more space.
- When the front is blocked, it compares left and right distance and turns
  toward the more open side.
- If all three directions are blocked, it reverses and then turns toward the
  freer side.
- Ultrasonic sensors are triggered one at a time to reduce acoustic cross-talk.

## Import and build

1. Extract the ZIP.
2. In STM32CubeIDE select:
   `File > Import > General > Existing Projects into Workspace`
3. Select the extracted `Auto_Car_Project` folder.
4. Run `Project > Clean`.
5. Run `Project > Build Project`.
6. Flash and run on the NUCLEO-F411RE.

The car boots stopped. Place it safely in the maze and press the blue button.

## Tuning

Navigation constants are in:

`Core/Inc/car_config.h`

The most useful values are:

- `FRONT_STOP_MM`
- `FRONT_CLEAR_MM`
- `SIDE_BLOCKED_MM`
- `DRIVE_SPEED`
- `TURN_SPEED`
- `TURN_MIN_TIME_MS`
- `TURN_MAX_TIME_MS`

Start with the wheels lifted and verify that forward, reverse, left pivot, and
right pivot match the physical motor wiring.
