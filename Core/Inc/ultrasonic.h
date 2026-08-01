#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "cmsis_os.h"
#include <stdint.h>

typedef enum
{
    ULTRASONIC_FRONT = 0,
    ULTRASONIC_LEFT,
    ULTRASONIC_RIGHT,
    ULTRASONIC_COUNT
} UltrasonicId;

typedef struct
{
    uint16_t distance_mm[ULTRASONIC_COUNT];
    uint8_t valid[ULTRASONIC_COUNT];
    uint32_t updated_ms[ULTRASONIC_COUNT];
} UltrasonicData;

extern osMutexId_t ultrasonicMutexHandle;

void UltrasonicTask(void *argument);
void Ultrasonic_Copy(UltrasonicData *destination);

#endif
