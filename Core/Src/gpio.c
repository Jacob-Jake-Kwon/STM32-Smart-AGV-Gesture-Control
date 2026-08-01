#include "gpio.h"

void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    HAL_GPIO_WritePin(
        GPIOA,
        L298N_IN1_Pin | L298N_IN2_Pin | STATUS_LED_Pin,
        GPIO_PIN_RESET
    );

    HAL_GPIO_WritePin(
        GPIOB,
        L298N_IN3_Pin | L298N_IN4_Pin |
        US_FRONT_TRIG_Pin | US_LEFT_TRIG_Pin | US_RIGHT_TRIG_Pin,
        GPIO_PIN_RESET
    );

    GPIO_InitStruct.Pin = L298N_IN1_Pin | L298N_IN2_Pin | STATUS_LED_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin =
        L298N_IN3_Pin | L298N_IN4_Pin |
        US_FRONT_TRIG_Pin | US_LEFT_TRIG_Pin | US_RIGHT_TRIG_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin =
        US_FRONT_ECHO_Pin | US_LEFT_ECHO_Pin | US_RIGHT_ECHO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = USER_BUTTON_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);
}
