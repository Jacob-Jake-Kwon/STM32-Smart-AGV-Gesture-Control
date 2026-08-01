#include "mode_manager.h"
#include "car_config.h"
#include "main.h"
#include "motor.h"

static volatile CarMode currentMode = CAR_MODE_STOP;

CarMode ModeManager_GetMode(void)
{
    return currentMode;
}

/*
 * Blue-button mode sequence:
 *
 * Power on -> STOP
 * 1st press -> AUTO
 * 2nd press -> MANUAL
 * 3rd press -> STOP
 * Repeat
 *
 * LED:
 * STOP   = off
 * AUTO   = continuously on
 * MANUAL = blinking
 */
void ModeManagerTask(void *argument)
{
    (void)argument;

    uint8_t previous_pressed = 0U;
    uint32_t last_blink = HAL_GetTick();

    Motor_Init();
    Motor_Stop();

    currentMode = CAR_MODE_STOP;
    HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);

    for (;;)
    {
        uint32_t now = HAL_GetTick();
        uint8_t pressed =
            HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin)
            == GPIO_PIN_RESET;

        if ((pressed != 0U) && (previous_pressed == 0U))
        {
            osDelay(BUTTON_DEBOUNCE_MS);

            if (HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin)
                == GPIO_PIN_RESET)
            {
                currentMode = (CarMode)(currentMode + 1);

                if (currentMode >= CAR_MODE_COUNT)
                {
                    currentMode = CAR_MODE_STOP;
                }

                /*
                 * Always stop between modes so the previous task cannot leave
                 * an old motor command active.
                 */
                Motor_Stop();

                if (currentMode == CAR_MODE_STOP)
                {
                    HAL_GPIO_WritePin(
                        STATUS_LED_GPIO_Port,
                        STATUS_LED_Pin,
                        GPIO_PIN_RESET
                    );
                }
                else if (currentMode == CAR_MODE_AUTO)
                {
                    HAL_GPIO_WritePin(
                        STATUS_LED_GPIO_Port,
                        STATUS_LED_Pin,
                        GPIO_PIN_SET
                    );
                }
                else
                {
                    HAL_GPIO_WritePin(
                        STATUS_LED_GPIO_Port,
                        STATUS_LED_Pin,
                        GPIO_PIN_RESET
                    );
                    last_blink = now;
                }

                while (HAL_GPIO_ReadPin(
                           USER_BUTTON_GPIO_Port,
                           USER_BUTTON_Pin) == GPIO_PIN_RESET)
                {
                    osDelay(10U);
                }
            }
        }

        previous_pressed = pressed;

        if ((currentMode == CAR_MODE_MANUAL) &&
            ((now - last_blink) >= 250U))
        {
            HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
            last_blink = now;
        }

        osDelay(20U);
    }
}
