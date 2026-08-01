#include "ultrasonic.h"
#include "car_config.h"
#include "main.h"
#include "tim.h"
#include <string.h>

osMutexId_t ultrasonicMutexHandle;

static UltrasonicData data;

static uint32_t TimerNowUs(void)
{
    return __HAL_TIM_GET_COUNTER(&htim5);
}

static void DelayUs(uint32_t delay_us)
{
    uint32_t start = TimerNowUs();
    while ((TimerNowUs() - start) < delay_us)
    {
        __NOP();
    }
}

static void TriggerWrite(UltrasonicId id, GPIO_PinState state)
{
    switch (id)
    {
        case ULTRASONIC_FRONT:
            HAL_GPIO_WritePin(US_FRONT_TRIG_GPIO_Port, US_FRONT_TRIG_Pin, state);
            break;

        case ULTRASONIC_LEFT:
            HAL_GPIO_WritePin(US_LEFT_TRIG_GPIO_Port, US_LEFT_TRIG_Pin, state);
            break;

        case ULTRASONIC_RIGHT:
            HAL_GPIO_WritePin(US_RIGHT_TRIG_GPIO_Port, US_RIGHT_TRIG_Pin, state);
            break;

        default:
            break;
    }
}

static GPIO_PinState EchoRead(UltrasonicId id)
{
    switch (id)
    {
        case ULTRASONIC_FRONT:
            return HAL_GPIO_ReadPin(US_FRONT_ECHO_GPIO_Port, US_FRONT_ECHO_Pin);

        case ULTRASONIC_LEFT:
            return HAL_GPIO_ReadPin(US_LEFT_ECHO_GPIO_Port, US_LEFT_ECHO_Pin);

        case ULTRASONIC_RIGHT:
            return HAL_GPIO_ReadPin(US_RIGHT_ECHO_GPIO_Port, US_RIGHT_ECHO_Pin);

        default:
            return GPIO_PIN_RESET;
    }
}

static uint8_t Measure(UltrasonicId id, uint16_t *distance_mm)
{
    uint32_t timeout_start;
    uint32_t pulse_start;
    uint32_t pulse_us;
    uint32_t measured_mm;

    TriggerWrite(id, GPIO_PIN_RESET);
    DelayUs(3U);
    TriggerWrite(id, GPIO_PIN_SET);
    DelayUs(10U);
    TriggerWrite(id, GPIO_PIN_RESET);

    timeout_start = TimerNowUs();

    while (EchoRead(id) == GPIO_PIN_RESET)
    {
        if ((TimerNowUs() - timeout_start) >= ULTRASONIC_TIMEOUT_US)
        {
            return 0U;
        }
    }

    pulse_start = TimerNowUs();

    while (EchoRead(id) == GPIO_PIN_SET)
    {
        if ((TimerNowUs() - pulse_start) >= ULTRASONIC_TIMEOUT_US)
        {
            return 0U;
        }
    }

    pulse_us = TimerNowUs() - pulse_start;
    measured_mm = (pulse_us * 10U) / 58U;

    if ((measured_mm < ULTRASONIC_MIN_MM) ||
        (measured_mm > ULTRASONIC_MAX_MM))
    {
        return 0U;
    }

    *distance_mm = (uint16_t)measured_mm;
    return 1U;
}

void Ultrasonic_Copy(UltrasonicData *destination)
{
    if ((destination != NULL) &&
        (osMutexAcquire(ultrasonicMutexHandle, osWaitForever) == osOK))
    {
        memcpy(destination, &data, sizeof(data));
        osMutexRelease(ultrasonicMutexHandle);
    }
}

void UltrasonicTask(void *argument)
{
    (void)argument;
    UltrasonicId id;
    uint16_t measured_mm;

    memset(&data, 0, sizeof(data));

    if (HAL_TIM_Base_Start(&htim5) != HAL_OK)
    {
        Error_Handler();
    }

    for (;;)
    {
        for (id = ULTRASONIC_FRONT;
             id < ULTRASONIC_COUNT;
             id = (UltrasonicId)(id + 1))
        {
            uint8_t valid = Measure(id, &measured_mm);
            uint32_t now = HAL_GetTick();

            if (osMutexAcquire(ultrasonicMutexHandle, osWaitForever) == osOK)
            {
                data.valid[id] = valid;

                if (valid != 0U)
                {
                    if (data.distance_mm[id] == 0U)
                    {
                        data.distance_mm[id] = measured_mm;
                    }
                    else
                    {
                        /* 75% previous value + 25% new value. */
                        data.distance_mm[id] =
                            (uint16_t)((3U * data.distance_mm[id] +
                                      measured_mm) / 4U);
                    }

                    data.updated_ms[id] = now;
                }

                osMutexRelease(ultrasonicMutexHandle);
            }

            osDelay(ULTRASONIC_INTER_SENSOR_MS);
        }
    }
}
