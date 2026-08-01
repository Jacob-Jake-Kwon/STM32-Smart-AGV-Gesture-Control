#ifndef MODE_MANAGER_H
#define MODE_MANAGER_H

#include "cmsis_os.h"
#include <stdint.h>

typedef enum
{
    CAR_MODE_STOP = 0,
    CAR_MODE_AUTO,
    CAR_MODE_MANUAL,
    CAR_MODE_COUNT
} CarMode;

CarMode ModeManager_GetMode(void);
void ModeManagerTask(void *argument);

#endif
