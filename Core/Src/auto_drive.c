#include "auto_drive.h"
#include "car_config.h"
#include "main.h"
#include "motor.h"
#include "mode_manager.h"
#include "ultrasonic.h"

/*
 * ============================================================
 * CONFIRMED ROBOT MOVEMENT CONVENTION
 * ============================================================
 *
 * Commands are defined from the robot's own viewpoint:
 *
 * Motor_Set(DRIVE_SPEED, DRIVE_SPEED)
 *      -> Forward
 *
 * Motor_Set(-DRIVE_SPEED, -DRIVE_SPEED)
 *      -> Backward
 *
 * Motor_Set(TURN_SPEED, -TURN_SPEED)
 *      -> Rotate left
 *
 * Motor_Set(-TURN_SPEED, TURN_SPEED)
 *      -> Rotate right
 *
 * Motor_Set(0, DRIVE_SPEED)
 *      -> Curve right
 *
 * Motor_Set(DRIVE_SPEED, 0)
 *      -> Curve left
 *
 * Do not change these commands unless the motor wiring or
 * physical motor mounting changes.
 * ============================================================
 */

typedef enum
{
    DRIVE_FORWARD = 0,
    DRIVE_STOP_PAUSE,
    DRIVE_REVERSE,
    DRIVE_TURN_LEFT,
    DRIVE_TURN_RIGHT
} DriveState;

static uint8_t SensorFresh(
    const UltrasonicData *sensors,
    UltrasonicId id,
    uint32_t now)
{
    return (sensors->valid[id] != 0U) &&
           ((now - sensors->updated_ms[id]) <= ULTRASONIC_STALE_MS);
}

void AutoDriveTask(void *argument)
{
    (void)argument;

    DriveState state = DRIVE_FORWARD;

    uint32_t state_started = HAL_GetTick();
    uint32_t last_junction_turn = 0U;

    uint8_t preferred_left = 1U;

    Motor_Stop();

    for (;;)
    {
        UltrasonicData sensors;
        uint32_t now = HAL_GetTick();

        /*
         * Automatic mode disabled:
         * keep both motor sides stopped.
         */
        CarMode current_mode = ModeManager_GetMode();

        /*
         * STOP mode:
         * This task is allowed to stop the motors.
         */
        if (current_mode == CAR_MODE_STOP)
        {
            Motor_Stop();

            state = DRIVE_FORWARD;
            state_started = now;
            last_junction_turn = now;

            osDelay(30U);
            continue;
        }

        /*
         * MANUAL mode:
         * Do not issue any motor command here.
         *
         * ManualDriveTask owns the motors during MANUAL mode.
         * Calling Motor_Stop() here would overwrite Bluetooth commands.
         */
        if (current_mode == CAR_MODE_MANUAL)
        {
            state = DRIVE_FORWARD;
            state_started = now;
            last_junction_turn = now;

            osDelay(30U);
            continue;
        }

        /*
         * Only CAR_MODE_AUTO reaches the autonomous navigation logic.
         */

        Ultrasonic_Copy(&sensors);

        uint8_t front_valid =
            SensorFresh(
                &sensors,
                ULTRASONIC_FRONT,
                now
            );

        uint8_t left_valid =
            SensorFresh(
                &sensors,
                ULTRASONIC_LEFT,
                now
            );

        uint8_t right_valid =
            SensorFresh(
                &sensors,
                ULTRASONIC_RIGHT,
                now
            );

        uint16_t front =
            sensors.distance_mm[ULTRASONIC_FRONT];

        uint16_t left =
            sensors.distance_mm[ULTRASONIC_LEFT];

        uint16_t right =
            sensors.distance_mm[ULTRASONIC_RIGHT];

        /*
         * Front sensor must be valid for safe driving.
         */
        if (front_valid == 0U)
        {
            Motor_Stop();

            osDelay(30U);
            continue;
        }

        switch (state)
        {
            case DRIVE_FORWARD:
            {
                /*
                 * Front obstacle:
                 * stop and determine the more open side.
                 */
                if (front <= FRONT_STOP_MM)
                {
                    Motor_Stop();

                    if (left_valid && right_valid)
                    {
                        preferred_left = (left >= right);
                    }
                    else if (left_valid)
                    {
                        preferred_left = 1U;
                    }
                    else
                    {
                        preferred_left = 0U;
                    }

                    state = DRIVE_STOP_PAUSE;
                    state_started = now;
                }

                /*
                 * At a junction, turn toward a side only when that
                 * side is significantly more open than forward.
                 */
                else if (
                    ((now - last_junction_turn) >=
                     JUNCTION_COOLDOWN_MS) &&
                    left_valid &&
                    right_valid)
                {
                    if (
                        (left >= SIDE_OPEN_MM) &&
                        (left >
                         (uint16_t)(
                             front + JUNCTION_ADVANTAGE_MM
                         )) &&
                        (left > right))
                    {
                        /*
                         * Confirmed left rotation.
                         */
                        Motor_Set(
                            TURN_SPEED,
                            -TURN_SPEED
                        );

                        state = DRIVE_TURN_LEFT;
                        state_started = now;
                        last_junction_turn = now;
                    }
                    else if (
                        (right >= SIDE_OPEN_MM) &&
                        (right >
                         (uint16_t)(
                             front + JUNCTION_ADVANTAGE_MM
                         )) &&
                        (right > left))
                    {
                        /*
                         * Confirmed right rotation.
                         */
                        Motor_Set(
                            -TURN_SPEED,
                            TURN_SPEED
                        );

                        state = DRIVE_TURN_RIGHT;
                        state_started = now;
                        last_junction_turn = now;
                    }
                    else if (left < SIDE_BLOCKED_MM)
                    {
                        /*
                         * Left wall is close:
                         * stop left wheels and curve right.
                         */
                        Motor_Set(
                            0,
                            DRIVE_SPEED
                        );
                    }
                    else if (right < SIDE_BLOCKED_MM)
                    {
                        /*
                         * Right wall is close:
                         * stop right wheels and curve left.
                         */
                        Motor_Set(
                            DRIVE_SPEED,
                            0
                        );
                    }
                    else
                    {
                        Motor_Set(
                            DRIVE_SPEED,
                            DRIVE_SPEED
                        );
                    }
                }

                /*
                 * Corridor wall correction.
                 */
                else if (
                    left_valid &&
                    (left < SIDE_BLOCKED_MM))
                {
                    /*
                     * Obstacle on robot's left:
                     * curve right.
                     */
                    Motor_Set(
                        0,
                        DRIVE_SPEED
                    );
                }
                else if (
                    right_valid &&
                    (right < SIDE_BLOCKED_MM))
                {
                    /*
                     * Obstacle on robot's right:
                     * curve left.
                     */
                    Motor_Set(
                        DRIVE_SPEED,
                        0
                    );
                }
                else
                {
                    Motor_Set(
                        DRIVE_SPEED,
                        DRIVE_SPEED
                    );
                }

                break;
            }

            case DRIVE_STOP_PAUSE:
            {
                Motor_Stop();

                if ((now - state_started) >= STOP_PAUSE_MS)
                {
                    uint8_t left_blocked =
                        (left_valid == 0U) ||
                        (left < SIDE_BLOCKED_MM);

                    uint8_t right_blocked =
                        (right_valid == 0U) ||
                        (right < SIDE_BLOCKED_MM);

                    /*
                     * If both sides are blocked, reverse first.
                     */
                    if (left_blocked && right_blocked)
                    {
                        Motor_Set(
                            -REVERSE_SPEED,
                            -REVERSE_SPEED
                        );

                        state = DRIVE_REVERSE;
                    }
                    else if (preferred_left != 0U)
                    {
                        /*
                         * Rotate left.
                         */
                        Motor_Set(
                            TURN_SPEED,
                            -TURN_SPEED
                        );

                        state = DRIVE_TURN_LEFT;
                    }
                    else
                    {
                        /*
                         * Rotate right.
                         */
                        Motor_Set(
                            -TURN_SPEED,
                            TURN_SPEED
                        );

                        state = DRIVE_TURN_RIGHT;
                    }

                    state_started = now;
                }

                break;
            }

            case DRIVE_REVERSE:
            {
                Motor_Set(
                    -REVERSE_SPEED,
                    -REVERSE_SPEED
                );

                if ((now - state_started) >= REVERSE_TIME_MS)
                {
                    if (left_valid && right_valid)
                    {
                        preferred_left = (left >= right);
                    }
                    else if (left_valid)
                    {
                        preferred_left = 1U;
                    }
                    else
                    {
                        preferred_left = 0U;
                    }

                    if (preferred_left != 0U)
                    {
                        Motor_Set(
                            TURN_SPEED,
                            -TURN_SPEED
                        );

                        state = DRIVE_TURN_LEFT;
                    }
                    else
                    {
                        Motor_Set(
                            -TURN_SPEED,
                            TURN_SPEED
                        );

                        state = DRIVE_TURN_RIGHT;
                    }

                    state_started = now;
                }

                break;
            }

            case DRIVE_TURN_LEFT:
            {
                Motor_Set(
                    TURN_SPEED,
                    -TURN_SPEED
                );

                /*
                 * Continue turning until the front becomes clear,
                 * but stop after the maximum permitted turn time.
                 */
                if (
                    (((now - state_started) >= TURN_MIN_TIME_MS) &&
                     (front >= FRONT_CLEAR_MM)) ||
                    ((now - state_started) >= TURN_MAX_TIME_MS))
                {
                    Motor_Stop();

                    state = DRIVE_FORWARD;
                    state_started = now;
                    last_junction_turn = now;
                }

                break;
            }

            case DRIVE_TURN_RIGHT:
            {
                Motor_Set(
                    -TURN_SPEED,
                    TURN_SPEED
                );

                if (
                    (((now - state_started) >= TURN_MIN_TIME_MS) &&
                     (front >= FRONT_CLEAR_MM)) ||
                    ((now - state_started) >= TURN_MAX_TIME_MS))
                {
                    Motor_Stop();

                    state = DRIVE_FORWARD;
                    state_started = now;
                    last_junction_turn = now;
                }

                break;
            }

            default:
            {
                Motor_Stop();

                state = DRIVE_FORWARD;
                state_started = now;
                last_junction_turn = now;

                break;
            }
        }

        osDelay(30U);
    }
}
