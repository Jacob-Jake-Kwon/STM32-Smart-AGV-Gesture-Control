#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include "cmsis_os.h"
#include <stdint.h>

typedef enum
{
    MANUAL_CMD_STOP = 0,
    MANUAL_CMD_FORWARD,
    MANUAL_CMD_BACKWARD,
    MANUAL_CMD_LEFT,
    MANUAL_CMD_RIGHT,
    MANUAL_CMD_FORWARD_LEFT,
    MANUAL_CMD_FORWARD_RIGHT,
    MANUAL_CMD_BACKWARD_LEFT,
    MANUAL_CMD_BACKWARD_RIGHT
} ManualCommand;

ManualCommand Bluetooth_GetCommand(void);
uint32_t Bluetooth_GetLastCommandTime(void);
void BluetoothTask(void *argument);

#endif
