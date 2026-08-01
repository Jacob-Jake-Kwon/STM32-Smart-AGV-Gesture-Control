#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

void Error_Handler(void);

/* L298N motor driver */
#define L298N_IN1_Pin               GPIO_PIN_9
#define L298N_IN1_GPIO_Port         GPIOA
#define L298N_IN2_Pin               GPIO_PIN_10
#define L298N_IN2_GPIO_Port         GPIOA

#define L298N_IN3_Pin               GPIO_PIN_5
#define L298N_IN3_GPIO_Port         GPIOB
#define L298N_IN4_Pin               GPIO_PIN_6
#define L298N_IN4_GPIO_Port         GPIOB

/* ENA = PA8 / TIM1_CH1 */
/* ENB = PB4 / TIM3_CH1 */

/* Ultrasonic triggers */
#define US_FRONT_TRIG_Pin           GPIO_PIN_0
#define US_FRONT_TRIG_GPIO_Port     GPIOB
#define US_LEFT_TRIG_Pin            GPIO_PIN_1
#define US_LEFT_TRIG_GPIO_Port      GPIOB
#define US_RIGHT_TRIG_Pin           GPIO_PIN_2
#define US_RIGHT_TRIG_GPIO_Port     GPIOB

/* Ultrasonic echoes */
#define US_FRONT_ECHO_Pin           GPIO_PIN_10
#define US_FRONT_ECHO_GPIO_Port     GPIOB
#define US_LEFT_ECHO_Pin            GPIO_PIN_12
#define US_LEFT_ECHO_GPIO_Port      GPIOB
#define US_RIGHT_ECHO_Pin           GPIO_PIN_13
#define US_RIGHT_ECHO_GPIO_Port     GPIOB


/*
 * HC-05 / HC-06 Bluetooth serial module
 *
 * USART2 PA2/PA3 is intentionally NOT used because those pins are connected
 * to the NUCLEO ST-LINK virtual COM port and can electrically conflict with
 * the HC-05 TX signal.
 *
 * Bluetooth uses USART6 instead:
 *
 * HC-05 TXD -> PC7 / USART6_RX
 * HC-05 RXD <- PC6 / USART6_TX
 */
#define BT_UART_TX_Pin               GPIO_PIN_6
#define BT_UART_TX_GPIO_Port         GPIOC
#define BT_UART_RX_Pin               GPIO_PIN_7
#define BT_UART_RX_GPIO_Port         GPIOC

/* Nucleo controls */
#define STATUS_LED_Pin              GPIO_PIN_5
#define STATUS_LED_GPIO_Port        GPIOA
#define USER_BUTTON_Pin             GPIO_PIN_13
#define USER_BUTTON_GPIO_Port       GPIOC

#ifdef __cplusplus
}
#endif

#endif
