#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

void Motor_Init(void);
void Motor_Set(int16_t left_percent, int16_t right_percent);
void Motor_Stop(void);

#endif
