#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "auto_drive.h"
#include "bluetooth.h"
#include "manual_drive.h"
#include "mode_manager.h"
#include "ultrasonic.h"

osThreadId_t ultrasonicTaskHandle;
osThreadId_t autoDriveTaskHandle;
osThreadId_t manualDriveTaskHandle;
osThreadId_t bluetoothTaskHandle;
osThreadId_t modeManagerTaskHandle;

static const osThreadAttr_t ultrasonicTask_attributes = {
    .name = "UltrasonicTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

static const osThreadAttr_t autoDriveTask_attributes = {
    .name = "AutoDriveTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
};

static const osThreadAttr_t manualDriveTask_attributes = {
    .name = "ManualDriveTask",
    .stack_size = 384 * 4,
    .priority = (osPriority_t)osPriorityAboveNormal,
};

static const osThreadAttr_t bluetoothTask_attributes = {
    .name = "BluetoothTask",
    .stack_size = 384 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

static const osThreadAttr_t modeManagerTask_attributes = {
    .name = "ModeManagerTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityHigh,
};

void MX_FREERTOS_Init(void)
{
    const osMutexAttr_t ultrasonicMutex_attributes = {
        .name = "ultrasonicMutex"
    };

    ultrasonicMutexHandle = osMutexNew(&ultrasonicMutex_attributes);

    if (ultrasonicMutexHandle == NULL)
    {
        Error_Handler();
    }

    ultrasonicTaskHandle =
        osThreadNew(UltrasonicTask, NULL, &ultrasonicTask_attributes);

    autoDriveTaskHandle =
        osThreadNew(AutoDriveTask, NULL, &autoDriveTask_attributes);

    manualDriveTaskHandle =
        osThreadNew(ManualDriveTask, NULL, &manualDriveTask_attributes);

    bluetoothTaskHandle =
        osThreadNew(BluetoothTask, NULL, &bluetoothTask_attributes);

    modeManagerTaskHandle =
        osThreadNew(ModeManagerTask, NULL, &modeManagerTask_attributes);

    if ((ultrasonicTaskHandle == NULL) ||
        (autoDriveTaskHandle == NULL) ||
        (manualDriveTaskHandle == NULL) ||
        (bluetoothTaskHandle == NULL) ||
        (modeManagerTaskHandle == NULL))
    {
        Error_Handler();
    }
}
