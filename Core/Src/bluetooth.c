#include "bluetooth.h"
#include "usart.h"
#include <ctype.h>
#include <string.h>

static volatile ManualCommand latestCommand = MANUAL_CMD_STOP;
static volatile uint32_t lastCommandTime = 0U;

ManualCommand Bluetooth_GetCommand(void)
{
    return latestCommand;
}

uint32_t Bluetooth_GetLastCommandTime(void)
{
    return lastCommandTime;
}

static void ProcessLine(char *line)
{
    size_t i;

    for (i = 0U; line[i] != '\0'; i++)
    {
        line[i] = (char)toupper((unsigned char)line[i]);
    }

    if (strcmp(line, "F") == 0)
        latestCommand = MANUAL_CMD_FORWARD;
    else if (strcmp(line, "B") == 0)
        latestCommand = MANUAL_CMD_BACKWARD;
    else if (strcmp(line, "L") == 0)
        latestCommand = MANUAL_CMD_LEFT;
    else if (strcmp(line, "R") == 0)
        latestCommand = MANUAL_CMD_RIGHT;
    else if (strcmp(line, "FL") == 0)
        latestCommand = MANUAL_CMD_FORWARD_LEFT;
    else if (strcmp(line, "FR") == 0)
        latestCommand = MANUAL_CMD_FORWARD_RIGHT;
    else if (strcmp(line, "BL") == 0)
        latestCommand = MANUAL_CMD_BACKWARD_LEFT;
    else if (strcmp(line, "BR") == 0)
        latestCommand = MANUAL_CMD_BACKWARD_RIGHT;
    else
        latestCommand = MANUAL_CMD_STOP;

    lastCommandTime = HAL_GetTick();
}

/*
 * HC-05/HC-06 transparent UART protocol.
 *
 * Send newline-terminated ASCII commands:
 * F, B, L, R, FL, FR, BL, BR, S
 *
 * Example:
 * "F\n"
 */
void BluetoothTask(void *argument)
{
    (void)argument;

    uint8_t received;
    char line[8];
    uint8_t index = 0U;

    latestCommand = MANUAL_CMD_STOP;
    lastCommandTime = HAL_GetTick();

    for (;;)
    {
        if (HAL_UART_Receive(&huart6, &received, 1U, 20U) == HAL_OK)
        {
            if ((received == '\n') || (received == '\r'))
            {
                if (index > 0U)
                {
                    line[index] = '\0';
                    ProcessLine(line);
                    index = 0U;
                }
            }
            else if (index < (sizeof(line) - 1U))
            {
                line[index++] = (char)received;
            }
            else
            {
                index = 0U;
                latestCommand = MANUAL_CMD_STOP;
                lastCommandTime = HAL_GetTick();
            }
        }

        osDelay(1U);
    }
}
