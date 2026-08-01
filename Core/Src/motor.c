#include "motor.h"
#include "main.h"
#include "tim.h"

/*
 * ============================================================
 * CONFIRMED MOTOR ORIENTATION
 * ============================================================
 *
 * Blue-button calibration results:
 *
 * LEFT_MOTOR_REVERSED = 1
 * RIGHT_MOTOR_REVERSED = 0
 * caused:
 *   Positive commands  -> backward
 *   Negative commands  -> forward
 *
 * Therefore both channel configurations must be inverted.
 *
 * Do not change these values unless the motor wiring or physical
 * motor mounting is changed.
 */
#define LEFT_MOTOR_REVERSED     0U
#define RIGHT_MOTOR_REVERSED    1U

/*
 * Minimum usable PWM for the L298N and two TT motors per side.
 *
 * Commands below this value are increased to this minimum so
 * the motors do not merely buzz without rotating.
 *
 * Set to 0 if you do not want a minimum PWM limit.
 */
#define MOTOR_MIN_PWM_PERCENT   65U

/*
 * Maximum allowed PWM.
 *
 * Reduce this value to limit the overall maximum motor strength.
 * Example:
 *   100 = full available output
 *    80 = maximum output limited to 80%
 */
#define MOTOR_MAX_PWM_PERCENT   80U

/*
 * Stop behavior:
 *
 * 0 = coast stop:
 *     IN1 = LOW, IN2 = LOW
 *
 * 1 = active brake:
 *     IN1 = HIGH, IN2 = HIGH
 *
 * Coast is gentler for the gearbox and is recommended initially.
 */
#define MOTOR_USE_ACTIVE_BRAKE  0U


static int16_t Motor_ClampCommand(int16_t speed)
{
    if (speed > 100)
    {
        return 100;
    }

    if (speed < -100)
    {
        return -100;
    }

    return speed;
}


static uint16_t Motor_LimitMagnitude(uint16_t magnitude)
{
    if (magnitude == 0U)
    {
        return 0U;
    }

    /*
     * Ensure enough PWM to overcome the L298N voltage drop and
     * the starting friction of two motors connected in parallel.
     */
    if (magnitude < MOTOR_MIN_PWM_PERCENT)
    {
        magnitude = MOTOR_MIN_PWM_PERCENT;
    }

    /*
     * Limit maximum speed/strength.
     */
    if (magnitude > MOTOR_MAX_PWM_PERCENT)
    {
        magnitude = MOTOR_MAX_PWM_PERCENT;
    }

    return magnitude;
}


static uint32_t Motor_PercentToCompare(
    TIM_HandleTypeDef *timer,
    uint16_t percent)
{
    uint32_t timer_period;

    if (timer == NULL)
    {
        return 0U;
    }

    if (percent > 100U)
    {
        percent = 100U;
    }

    timer_period = __HAL_TIM_GET_AUTORELOAD(timer) + 1U;

    return (timer_period * percent) / 100U;
}


static void Motor_SetLeftDirection(int8_t direction)
{
    /*
     * direction:
     *  +1 = car forward
     *  -1 = car backward
     *   0 = stop
     */

#if LEFT_MOTOR_REVERSED
    direction = (int8_t)(-direction);
#endif

    if (direction > 0)
    {
        /*
         * Electrical direction A:
         * IN1 = HIGH
         * IN2 = LOW
         */
        HAL_GPIO_WritePin(
            L298N_IN1_GPIO_Port,
            L298N_IN1_Pin,
            GPIO_PIN_SET
        );

        HAL_GPIO_WritePin(
            L298N_IN2_GPIO_Port,
            L298N_IN2_Pin,
            GPIO_PIN_RESET
        );
    }
    else if (direction < 0)
    {
        /*
         * Electrical direction B:
         * IN1 = LOW
         * IN2 = HIGH
         */
        HAL_GPIO_WritePin(
            L298N_IN1_GPIO_Port,
            L298N_IN1_Pin,
            GPIO_PIN_RESET
        );

        HAL_GPIO_WritePin(
            L298N_IN2_GPIO_Port,
            L298N_IN2_Pin,
            GPIO_PIN_SET
        );
    }
    else
    {
#if MOTOR_USE_ACTIVE_BRAKE
        HAL_GPIO_WritePin(
            L298N_IN1_GPIO_Port,
            L298N_IN1_Pin,
            GPIO_PIN_SET
        );

        HAL_GPIO_WritePin(
            L298N_IN2_GPIO_Port,
            L298N_IN2_Pin,
            GPIO_PIN_SET
        );
#else
        HAL_GPIO_WritePin(
            L298N_IN1_GPIO_Port,
            L298N_IN1_Pin,
            GPIO_PIN_RESET
        );

        HAL_GPIO_WritePin(
            L298N_IN2_GPIO_Port,
            L298N_IN2_Pin,
            GPIO_PIN_RESET
        );
#endif
    }
}


static void Motor_SetRightDirection(int8_t direction)
{
    /*
     * direction:
     *  +1 = car forward
     *  -1 = car backward
     *   0 = stop
     */

#if RIGHT_MOTOR_REVERSED
    direction = (int8_t)(-direction);
#endif

    if (direction > 0)
    {
        /*
         * Electrical direction A:
         * IN3 = HIGH
         * IN4 = LOW
         */
        HAL_GPIO_WritePin(
            L298N_IN3_GPIO_Port,
            L298N_IN3_Pin,
            GPIO_PIN_SET
        );

        HAL_GPIO_WritePin(
            L298N_IN4_GPIO_Port,
            L298N_IN4_Pin,
            GPIO_PIN_RESET
        );
    }
    else if (direction < 0)
    {
        /*
         * Electrical direction B:
         * IN3 = LOW
         * IN4 = HIGH
         */
        HAL_GPIO_WritePin(
            L298N_IN3_GPIO_Port,
            L298N_IN3_Pin,
            GPIO_PIN_RESET
        );

        HAL_GPIO_WritePin(
            L298N_IN4_GPIO_Port,
            L298N_IN4_Pin,
            GPIO_PIN_SET
        );
    }
    else
    {
#if MOTOR_USE_ACTIVE_BRAKE
        HAL_GPIO_WritePin(
            L298N_IN3_GPIO_Port,
            L298N_IN3_Pin,
            GPIO_PIN_SET
        );

        HAL_GPIO_WritePin(
            L298N_IN4_GPIO_Port,
            L298N_IN4_Pin,
            GPIO_PIN_SET
        );
#else
        HAL_GPIO_WritePin(
            L298N_IN3_GPIO_Port,
            L298N_IN3_Pin,
            GPIO_PIN_RESET
        );

        HAL_GPIO_WritePin(
            L298N_IN4_GPIO_Port,
            L298N_IN4_Pin,
            GPIO_PIN_RESET
        );
#endif
    }
}


static void Motor_SetLeft(int16_t speed)
{
    uint16_t magnitude;
    int8_t direction;

    speed = Motor_ClampCommand(speed);

    if (speed > 0)
    {
        /*
         * Positive command always means CAR FORWARD.
         * Physical direction reversal is handled internally.
         */
        direction = 1;
        magnitude = (uint16_t)speed;
    }
    else if (speed < 0)
    {
        /*
         * Negative command always means CAR BACKWARD.
         */
        direction = -1;
        magnitude = (uint16_t)(-speed);
    }
    else
    {
        direction = 0;
        magnitude = 0U;
    }

    magnitude = Motor_LimitMagnitude(magnitude);

    Motor_SetLeftDirection(direction);

    __HAL_TIM_SET_COMPARE(
        &htim1,
        TIM_CHANNEL_1,
        Motor_PercentToCompare(&htim1, magnitude)
    );
}


static void Motor_SetRight(int16_t speed)
{
    uint16_t magnitude;
    int8_t direction;

    speed = Motor_ClampCommand(speed);

    if (speed > 0)
    {
        /*
         * Positive command always means CAR FORWARD.
         * Physical direction reversal is handled internally.
         */
        direction = 1;
        magnitude = (uint16_t)speed;
    }
    else if (speed < 0)
    {
        /*
         * Negative command always means CAR BACKWARD.
         */
        direction = -1;
        magnitude = (uint16_t)(-speed);
    }
    else
    {
        direction = 0;
        magnitude = 0U;
    }

    magnitude = Motor_LimitMagnitude(magnitude);

    Motor_SetRightDirection(direction);

    __HAL_TIM_SET_COMPARE(
        &htim3,
        TIM_CHANNEL_1,
        Motor_PercentToCompare(&htim3, magnitude)
    );
}


void Motor_Init(void)
{
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }

    __HAL_TIM_SET_COMPARE(
        &htim1,
        TIM_CHANNEL_1,
        0U
    );

    __HAL_TIM_SET_COMPARE(
        &htim3,
        TIM_CHANNEL_1,
        0U
    );

    Motor_Stop();
}


void Motor_Set(
    int16_t left_percent,
    int16_t right_percent)
{
    /*
     * IMPORTANT COMMAND CONVENTION
     * ========================================================
     *
     * Positive value  = car moves forward
     * Negative value  = car moves backward
     * Zero            = motor side stops
     *
     * Examples:
     *
     * Motor_Set(70, 70);
     *     Forward
     *
     * Motor_Set(-70, -70);
     *     Reverse
     *
     * Motor_Set(-70, 70);
     *     Rotate left
     *
     * Motor_Set(70, -70);
     *     Rotate right
     *
     * Motor_Set(0, 70);
     *     Stop left wheels and turn right
     *
     * Motor_Set(70, 0);
     *     Stop right wheels and turn left
     *
     * Do not reverse these command meanings elsewhere in the
     * navigation code. Physical direction differences must be
     * configured only with LEFT_MOTOR_REVERSED and
     * RIGHT_MOTOR_REVERSED at the top of this file.
     */

    Motor_SetLeft(left_percent);
    Motor_SetRight(right_percent);
}


void Motor_Stop(void)
{
    Motor_SetLeft(0);
    Motor_SetRight(0);
}
