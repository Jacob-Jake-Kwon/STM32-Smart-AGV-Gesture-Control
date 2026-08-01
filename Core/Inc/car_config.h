#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H

/* Ultrasonic timing */
#define ULTRASONIC_TIMEOUT_US             30000U
#define ULTRASONIC_INTER_SENSOR_MS           28U
#define ULTRASONIC_MIN_MM                    25U
#define ULTRASONIC_MAX_MM                  3000U
#define ULTRASONIC_STALE_MS                 450U

/* Maze-navigation thresholds */
#define FRONT_STOP_MM                       210U
#define FRONT_CLEAR_MM                      380U
#define SIDE_BLOCKED_MM                     300U
#define SIDE_OPEN_MM                        450U
#define JUNCTION_ADVANTAGE_MM               180U

/* L298N speeds. High values help four TT motors overcome starting friction. */
#define DRIVE_SPEED                          72
#define CORRECTION_SLOW_SPEED                65
#define TURN_SPEED                           75
#define REVERSE_SPEED                        70

/* State timing */
#define STOP_PAUSE_MS                       120U
#define REVERSE_TIME_MS                     520U
#define TURN_MIN_TIME_MS                    320U
#define TURN_MAX_TIME_MS                   1250U
#define JUNCTION_COOLDOWN_MS               1500U

/* Button */
#define BUTTON_DEBOUNCE_MS                   50U



/* Bluetooth serial module */
#define BLUETOOTH_BAUD_RATE                 9600U
#define BLUETOOTH_COMMAND_TIMEOUT_MS         600U

/* Manual mode motor settings */
#define MANUAL_DRIVE_SPEED                    72
#define MANUAL_TURN_SPEED                     75
#define MANUAL_CURVE_INNER_SPEED              65
#define MANUAL_FRONT_SAFETY_MM               210U

#endif
