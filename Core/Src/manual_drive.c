#include "main.h"

#include "manual_drive.h"
#include "bluetooth.h"
#include "car_config.h"
#include "mode_manager.h"
#include "motor.h"
#include "ultrasonic.h"

static uint8_t FrontSensorSafe(const UltrasonicData *sensors, uint32_t now)
{
    return (sensors->valid[ULTRASONIC_FRONT] != 0U) &&
           ((now - sensors->updated_ms[ULTRASONIC_FRONT])
            <= ULTRASONIC_STALE_MS) &&
           (sensors->distance_mm[ULTRASONIC_FRONT]
            > MANUAL_FRONT_SAFETY_MM);
}

void ManualDriveTask(void *argument)
{
    (void)argument;

    for (;;)
    {
        uint32_t now = HAL_GetTick();

        if (ModeManager_GetMode() != CAR_MODE_MANUAL)
        {
            osDelay(20U);
            continue;
        }

        ManualCommand command = Bluetooth_GetCommand();
        uint32_t last_command = Bluetooth_GetLastCommandTime();
        UltrasonicData sensors;

        /*
         * Radio failsafe: stop if commands cease.
         */
        if ((now - last_command) > BLUETOOTH_COMMAND_TIMEOUT_MS)
        {
            Motor_Stop();
            osDelay(20U);
            continue;
        }

        Ultrasonic_Copy(&sensors);
        uint8_t forward_safe = FrontSensorSafe(&sensors, now);

        switch (command)
        {
            case MANUAL_CMD_FORWARD:
                if (forward_safe)
                    Motor_Set(MANUAL_DRIVE_SPEED, MANUAL_DRIVE_SPEED);
                else
                    Motor_Stop();
                break;

            case MANUAL_CMD_BACKWARD:
                Motor_Set(-MANUAL_DRIVE_SPEED, -MANUAL_DRIVE_SPEED);
                break;

            case MANUAL_CMD_LEFT:
                Motor_Set(MANUAL_TURN_SPEED, -MANUAL_TURN_SPEED);
                break;

            case MANUAL_CMD_RIGHT:
                Motor_Set(-MANUAL_TURN_SPEED, MANUAL_TURN_SPEED);
                break;

            case MANUAL_CMD_FORWARD_LEFT:
                if (forward_safe)
                    Motor_Set(MANUAL_CURVE_INNER_SPEED, MANUAL_DRIVE_SPEED);
                else
                    Motor_Stop();
                break;

            case MANUAL_CMD_FORWARD_RIGHT:
                if (forward_safe)
                    Motor_Set(MANUAL_DRIVE_SPEED, MANUAL_CURVE_INNER_SPEED);
                else
                    Motor_Stop();
                break;

            case MANUAL_CMD_BACKWARD_LEFT:
                Motor_Set(-MANUAL_CURVE_INNER_SPEED, -MANUAL_DRIVE_SPEED);
                break;

            case MANUAL_CMD_BACKWARD_RIGHT:
                Motor_Set(-MANUAL_DRIVE_SPEED, -MANUAL_CURVE_INNER_SPEED);
                break;

            case MANUAL_CMD_STOP:
            default:
                Motor_Stop();
                break;
        }

        osDelay(20U);
    }
}
