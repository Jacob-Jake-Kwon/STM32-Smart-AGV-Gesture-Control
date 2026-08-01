#ifndef __TIM_H
#define __TIM_H

#include "main.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;

void MX_TIM1_Init(void);
void MX_TIM3_Init(void);
void MX_TIM5_Init(void);

#endif
