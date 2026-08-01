/*
 * ============================================================
 * ESP32 + MPU6050 HAND-GESTURE CONTROLLER FOR STM32 CAR
 * ============================================================
 *
 * Controller hardware:
 *   - Original ESP32 / ESP32-WROOM-32 board
 *   - MPU6050 / GY-521
 *
 * Robot-side wireless hardware:
 *   - HC-05 connected to STM32 USART2
 *
 * Bluetooth mode:
 *   - ESP32 = Bluetooth Classic master
 *   - HC-05 = Bluetooth Classic slave
 *
 * HC-05 default settings:
 *   Name: HC-05
 *   PIN : 1234
 *
 * Commands sent to STM32:
 *
 *   F\n   Forward
 *   B\n   Backward
 *   L\n   Rotate left
 *   R\n   Rotate right
 *   FL\n  Forward-left
 *   FR\n  Forward-right
 *   BL\n  Backward-left
 *   BR\n  Backward-right
 *   S\n   Stop
 *
 * Wiring:
 *
 *   MPU6050 VCC -> ESP32 3.3V
 *   MPU6050 GND -> ESP32 GND
 *   MPU6050 SDA -> GPIO21
 *   MPU6050 SCL -> GPIO22
 *   MPU6050 AD0 -> GND
 *
 * Keep the controller flat and motionless during startup
 * calibration.
 * ============================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <BluetoothSerial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>

/*
 * Bluetooth Classic is available on the original ESP32.
 */
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled for the selected ESP32 board.
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Bluetooth Classic SPP is not available. Select an original ESP32 board.
#endif

/* ============================================================
 * HC-05 CONFIGURATION
 * ============================================================
 */

static const char *HC05_NAME = "HC-05";
static const char *HC05_PIN  = "1234";

/*
 * Name advertised locally by the ESP32 controller.
 */
static const char *ESP32_BT_NAME = "ESP32_Hand_Controller";

/* ============================================================
 * MPU6050 CONFIGURATION
 * ============================================================
 */

static constexpr uint8_t MPU_SDA_PIN = 21U;
static constexpr uint8_t MPU_SCL_PIN = 22U;

/*
 * AD0 connected to GND gives address 0x68.
 */
static constexpr uint8_t MPU6050_ADDRESS = 0x68U;

/*
 * Gesture activation thresholds.
 *
 * Increase these if the controller reacts too easily.
 * Decrease them if greater hand tilt is required.
 */
static constexpr float PITCH_START_DEG = 14.0f;
static constexpr float ROLL_START_DEG  = 14.0f;

/*
 * Release thresholds provide hysteresis and reduce rapid
 * switching near the neutral position.
 */
static constexpr float PITCH_RELEASE_DEG = 8.0f;
static constexpr float ROLL_RELEASE_DEG  = 8.0f;

/*
 * Sensor filtering:
 *
 * Higher value = smoother but slower response.
 * Lower value  = faster but noisier response.
 */
static constexpr float FILTER_ALPHA = 0.88f;

/*
 * Timing configuration.
 */
static constexpr uint32_t SENSOR_INTERVAL_MS    = 20U;
static constexpr uint32_t COMMAND_INTERVAL_MS   = 100U;
static constexpr uint32_t RECONNECT_INTERVAL_MS = 3000U;

/*
 * Calibration takes approximately 1.5 seconds after the
 * initial two-second preparation delay.
 */
static constexpr uint16_t CALIBRATION_SAMPLES = 150U;

/*
 * Enable combined forward-left, forward-right, etc.
 */
static constexpr bool ENABLE_DIAGONAL_COMMANDS = true;

/*
 * ============================================================
 * SENSOR ORIENTATION
 * ============================================================
 *
 * Change only these values if the hand gestures are reversed.
 *
 * PITCH_DIRECTION:
 *   +1.0 = current forward/backward mapping
 *   -1.0 = reverse forward/backward mapping
 *
 * ROLL_DIRECTION:
 *   +1.0 = current left/right mapping
 *   -1.0 = reverse left/right mapping
 *
 * Do not reverse command strings elsewhere in the code.
 */
static constexpr float PITCH_DIRECTION = 1.0f;
static constexpr float ROLL_DIRECTION  = 1.0f;

/* ============================================================
 * GLOBAL OBJECTS
 * ============================================================
 */

BluetoothSerial SerialBT;
Adafruit_MPU6050 mpu;

enum class GestureCommand : uint8_t
{
    Stop = 0,
    Forward,
    Backward,
    Left,
    Right,
    ForwardLeft,
    ForwardRight,
    BackwardLeft,
    BackwardRight
};

static float neutralPitchDeg = 0.0f;
static float neutralRollDeg  = 0.0f;

static float filteredPitchDeg = 0.0f;
static float filteredRollDeg  = 0.0f;

static bool pitchActive = false;
static bool rollActive  = false;

static GestureCommand currentCommand  = GestureCommand::Stop;
static GestureCommand previousCommand = GestureCommand::Stop;

static uint32_t previousSensorMs    = 0U;
static uint32_t previousCommandMs   = 0U;
static uint32_t previousReconnectMs = 0U;

/* ============================================================
 * COMMAND CONVERSION
 * ============================================================
 */

static const char *commandToPacket(GestureCommand command)
{
    switch (command)
    {
        case GestureCommand::Forward:
            return "F\n";

        case GestureCommand::Backward:
            return "B\n";

        case GestureCommand::Left:
            return "L\n";

        case GestureCommand::Right:
            return "R\n";

        case GestureCommand::ForwardLeft:
            return "FL\n";

        case GestureCommand::ForwardRight:
            return "FR\n";

        case GestureCommand::BackwardLeft:
            return "BL\n";

        case GestureCommand::BackwardRight:
            return "BR\n";

        case GestureCommand::Stop:
        default:
            return "S\n";
    }
}

static const char *commandToText(GestureCommand command)
{
    switch (command)
    {
        case GestureCommand::Forward:
            return "FORWARD";

        case GestureCommand::Backward:
            return "BACKWARD";

        case GestureCommand::Left:
            return "LEFT";

        case GestureCommand::Right:
            return "RIGHT";

        case GestureCommand::ForwardLeft:
            return "FORWARD LEFT";

        case GestureCommand::ForwardRight:
            return "FORWARD RIGHT";

        case GestureCommand::BackwardLeft:
            return "BACKWARD LEFT";

        case GestureCommand::BackwardRight:
            return "BACKWARD RIGHT";

        case GestureCommand::Stop:
        default:
            return "STOP";
    }
}

/* ============================================================
 * MPU6050 ANGLE PROCESSING
 * ============================================================
 */

static void calculateAngles(
    const sensors_event_t &acceleration,
    float &pitchDeg,
    float &rollDeg)
{
    const float ax = acceleration.acceleration.x;
    const float ay = acceleration.acceleration.y;
    const float az = acceleration.acceleration.z;

    /*
     * Pitch:
     * rotation forward/backward.
     */
    pitchDeg =
        atan2f(
            -ax,
            sqrtf((ay * ay) + (az * az))
        ) *
        180.0f / PI;

    /*
     * Roll:
     * rotation left/right.
     */
    rollDeg =
        atan2f(ay, az) *
        180.0f / PI;
}

static bool readRelativeAngles(
    float &relativePitchDeg,
    float &relativeRollDeg)
{
    sensors_event_t acceleration;
    sensors_event_t gyroscope;
    sensors_event_t temperature;

    if (!mpu.getEvent(
            &acceleration,
            &gyroscope,
            &temperature))
    {
        return false;
    }

    float pitchDeg;
    float rollDeg;

    calculateAngles(
        acceleration,
        pitchDeg,
        rollDeg
    );

    pitchDeg =
        (pitchDeg - neutralPitchDeg) *
        PITCH_DIRECTION;

    rollDeg =
        (rollDeg - neutralRollDeg) *
        ROLL_DIRECTION;

    filteredPitchDeg =
        (FILTER_ALPHA * filteredPitchDeg) +
        ((1.0f - FILTER_ALPHA) * pitchDeg);

    filteredRollDeg =
        (FILTER_ALPHA * filteredRollDeg) +
        ((1.0f - FILTER_ALPHA) * rollDeg);

    relativePitchDeg = filteredPitchDeg;
    relativeRollDeg  = filteredRollDeg;

    return true;
}

/* ============================================================
 * GESTURE DECISION
 * ============================================================
 */

static GestureCommand determineGesture(
    float pitchDeg,
    float rollDeg)
{
    /*
     * Pitch hysteresis.
     */
    if (!pitchActive)
    {
        if (fabsf(pitchDeg) >= PITCH_START_DEG)
        {
            pitchActive = true;
        }
    }
    else if (fabsf(pitchDeg) <= PITCH_RELEASE_DEG)
    {
        pitchActive = false;
    }

    /*
     * Roll hysteresis.
     */
    if (!rollActive)
    {
        if (fabsf(rollDeg) >= ROLL_START_DEG)
        {
            rollActive = true;
        }
    }
    else if (fabsf(rollDeg) <= ROLL_RELEASE_DEG)
    {
        rollActive = false;
    }

    const bool forward =
        pitchActive &&
        (pitchDeg > 0.0f);

    const bool backward =
        pitchActive &&
        (pitchDeg < 0.0f);

    const bool left =
        rollActive &&
        (rollDeg < 0.0f);

    const bool right =
        rollActive &&
        (rollDeg > 0.0f);

    if (ENABLE_DIAGONAL_COMMANDS)
    {
        if (forward && left)
        {
            return GestureCommand::ForwardLeft;
        }

        if (forward && right)
        {
            return GestureCommand::ForwardRight;
        }

        if (backward && left)
        {
            return GestureCommand::BackwardLeft;
        }

        if (backward && right)
        {
            return GestureCommand::BackwardRight;
        }
    }

    /*
     * If both axes are active and diagonal commands are disabled,
     * use whichever hand tilt is stronger.
     */
    if (pitchActive && rollActive)
    {
        if (fabsf(pitchDeg) >= fabsf(rollDeg))
        {
            return forward
                ? GestureCommand::Forward
                : GestureCommand::Backward;
        }

        return left
            ? GestureCommand::Left
            : GestureCommand::Right;
    }

    if (forward)
    {
        return GestureCommand::Forward;
    }

    if (backward)
    {
        return GestureCommand::Backward;
    }

    if (left)
    {
        return GestureCommand::Left;
    }

    if (right)
    {
        return GestureCommand::Right;
    }

    return GestureCommand::Stop;
}

/* ============================================================
 * MPU6050 CALIBRATION
 * ============================================================
 */

static bool calibrateNeutralPosition()
{
    Serial.println();
    Serial.println("========================================");
    Serial.println("MPU6050 neutral calibration");
    Serial.println("Hold the controller flat and motionless.");
    Serial.println("Calibration starts in two seconds.");
    Serial.println("========================================");

    delay(2000);

    float totalPitch = 0.0f;
    float totalRoll  = 0.0f;

    uint16_t successfulSamples = 0U;

    for (uint16_t sample = 0U;
         sample < CALIBRATION_SAMPLES;
         sample++)
    {
        sensors_event_t acceleration;
        sensors_event_t gyroscope;
        sensors_event_t temperature;

        if (mpu.getEvent(
                &acceleration,
                &gyroscope,
                &temperature))
        {
            float pitchDeg;
            float rollDeg;

            calculateAngles(
                acceleration,
                pitchDeg,
                rollDeg
            );

            totalPitch += pitchDeg;
            totalRoll  += rollDeg;

            successfulSamples++;
        }

        delay(10);
    }

    if (successfulSamples == 0U)
    {
        Serial.println(
            "Calibration failed: no MPU6050 readings."
        );

        return false;
    }

    neutralPitchDeg =
        totalPitch /
        static_cast<float>(successfulSamples);

    neutralRollDeg =
        totalRoll /
        static_cast<float>(successfulSamples);

    filteredPitchDeg = 0.0f;
    filteredRollDeg  = 0.0f;

    pitchActive = false;
    rollActive  = false;

    Serial.print("Neutral pitch: ");
    Serial.println(neutralPitchDeg, 2);

    Serial.print("Neutral roll: ");
    Serial.println(neutralRollDeg, 2);

    Serial.println("Calibration complete.");

    return true;
}

/* ============================================================
 * BLUETOOTH CONNECTION
 * ============================================================
 */

static bool connectToHC05()
{
    if (SerialBT.connected())
    {
        return true;
    }

    Serial.print("Connecting to ");
    Serial.print(HC05_NAME);
    Serial.println("...");

    /*
     * Apply the HC-05 legacy pairing PIN.
     */
    SerialBT.setPin(HC05_PIN);

    const bool connected =
        SerialBT.connect(HC05_NAME);

    if (!connected)
    {
        Serial.println("HC-05 connection failed.");

        return false;
    }

    Serial.println("HC-05 connected.");

    /*
     * Ensure the robot remains stopped immediately after
     * connecting or reconnecting.
     */
    SerialBT.print("S\n");

    currentCommand  = GestureCommand::Stop;
    previousCommand = GestureCommand::Stop;

    previousCommandMs = millis();

    return true;
}

static void sendCommand(
    GestureCommand command,
    bool forceSend)
{
    if (!SerialBT.connected())
    {
        return;
    }

    const uint32_t now = millis();

    const bool commandChanged =
        command != previousCommand;

    const bool repetitionDue =
        (now - previousCommandMs) >=
        COMMAND_INTERVAL_MS;

    if (forceSend ||
        commandChanged ||
        repetitionDue)
    {
        SerialBT.print(
            commandToPacket(command)
        );

        previousCommandMs = now;

        if (commandChanged)
        {
            Serial.print("Command: ");
            Serial.println(
                commandToText(command)
            );
        }

        previousCommand = command;
    }
}

/* ============================================================
 * ARDUINO SETUP
 * ============================================================
 */

void setup()
{
    Serial.begin(115200);

    delay(700);

    Serial.println();
    Serial.println(
        "ESP32 MPU6050 HC-05 Hand Controller"
    );

    /*
     * Start the MPU6050 I2C bus.
     */
    Wire.begin(
        MPU_SDA_PIN,
        MPU_SCL_PIN
    );

    Wire.setClock(400000U);

    if (!mpu.begin(
            MPU6050_ADDRESS,
            &Wire))
    {
        Serial.println("MPU6050 not detected.");
        Serial.println(
            "Check 3.3V, GND, GPIO21 SDA and GPIO22 SCL."
        );

        while (true)
        {
            delay(1000);
        }
    }

    Serial.println("MPU6050 detected.");

    mpu.setAccelerometerRange(
        MPU6050_RANGE_4_G
    );

    mpu.setGyroRange(
        MPU6050_RANGE_500_DEG
    );

    mpu.setFilterBandwidth(
        MPU6050_BAND_21_HZ
    );

    if (!calibrateNeutralPosition())
    {
        while (true)
        {
            delay(1000);
        }
    }

    /*
     * Start the ESP32 as a Bluetooth Classic master.
     */
    if (!SerialBT.begin(
            ESP32_BT_NAME,
            true))
    {
        Serial.println(
            "Bluetooth Classic initialization failed."
        );

        while (true)
        {
            delay(1000);
        }
    }

    Serial.println(
        "Bluetooth Classic master initialized."
    );

    /*
     * Clear old pairing records only when troubleshooting.
     *
     * Uncomment once, upload, run, then comment it again.
     */
    // SerialBT.deleteAllBondedDevices();

    connectToHC05();
}

/* ============================================================
 * ARDUINO LOOP
 * ============================================================
 */

void loop()
{
    const uint32_t now = millis();

    /*
     * Reconnect automatically after a disconnection.
     */
    if (!SerialBT.connected())
    {
        currentCommand = GestureCommand::Stop;

        if ((now - previousReconnectMs) >=
            RECONNECT_INTERVAL_MS)
        {
            previousReconnectMs = now;

            connectToHC05();
        }

        delay(20);

        return;
    }

    /*
     * Read the MPU6050 at 50 Hz.
     */
    if ((now - previousSensorMs) >=
        SENSOR_INTERVAL_MS)
    {
        previousSensorMs = now;

        float pitchDeg;
        float rollDeg;

        if (readRelativeAngles(
                pitchDeg,
                rollDeg))
        {
            currentCommand =
                determineGesture(
                    pitchDeg,
                    rollDeg
                );

            sendCommand(
                currentCommand,
                false
            );

            /*
             * Serial Monitor diagnostic output.
             */
            Serial.print("Pitch:");
            Serial.print(pitchDeg, 1);

            Serial.print(" Roll:");
            Serial.print(rollDeg, 1);

            Serial.print(" Command:");
            Serial.println(
                commandToText(currentCommand)
            );
        }
        else
        {
            /*
             * Stop the robot if the MPU6050 read fails.
             */
            currentCommand =
                GestureCommand::Stop;

            sendCommand(
                GestureCommand::Stop,
                true
            );

            Serial.println(
                "MPU6050 read failed: sent STOP."
            );
        }
    }

    delay(1);
}