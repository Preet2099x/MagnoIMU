#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stddef.h>
#include <EEPROM.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define BMM350_ADDR 0x14

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
bool bnoReady = false;

struct PersistedState
{
    uint32_t magic;
    uint16_t version;
    uint16_t checksum;
    float lastYawDeg;
    uint8_t yawValid;
    uint8_t bnoOffsetsValid;
    uint8_t reserved[2];
    adafruit_bno055_offsets_t bnoOffsets;
};

const uint32_t PERSIST_MAGIC = 0x4D494D55UL;
const uint16_t PERSIST_VERSION = 1;
const int EEPROM_ADDR = 0;
const unsigned long EEPROM_SAVE_INTERVAL_MS = 2000;
const float EEPROM_SAVE_MIN_DELTA_DEG = 0.5f;

unsigned long lastEepromSaveMs = 0;
float lastSavedYawDeg = 0.0f;
bool savedOffsetsAlready = false;

uint16_t computePersistChecksum(const PersistedState &state)
{
    const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&state);
    uint16_t sum = 0;

    for (size_t i = 0; i < sizeof(PersistedState); i++)
    {
        if (i == offsetof(PersistedState, checksum) || i == (offsetof(PersistedState, checksum) + 1))
            continue;
        sum = static_cast<uint16_t>(sum + bytes[i]);
    }

    return sum;
}

bool persistedStateValid(const PersistedState &state)
{
    if (state.magic != PERSIST_MAGIC)
        return false;
    if (state.version != PERSIST_VERSION)
        return false;
    return state.checksum == computePersistChecksum(state);
}

bool loadPersistedState(PersistedState &state)
{
    EEPROM.get(EEPROM_ADDR, state);
    return persistedStateValid(state);
}

void savePersistedState(PersistedState state)
{
    state.magic = PERSIST_MAGIC;
    state.version = PERSIST_VERSION;
    state.checksum = computePersistChecksum(state);
    EEPROM.put(EEPROM_ADDR, state);
}

/* I2C read (2 dummy bytes) */
bool readReg(uint8_t reg, uint8_t *data, uint8_t len)
{
    Wire.beginTransmission(BMM350_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0)
        return false;

    uint8_t total = len + 2;

    if (Wire.requestFrom(static_cast<uint8_t>(BMM350_ADDR), static_cast<uint8_t>(total)) != total)
        return false;

    Wire.read();
    Wire.read();

    for (uint8_t i = 0; i < len; i++)
        data[i] = Wire.read();

    return true;
}

/* I2C write */
bool writeReg(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(BMM350_ADDR);
    Wire.write(reg);
    Wire.write(value);
    return (Wire.endTransmission() == 0);
}

/* ---- Magnetic Reset ---- */
void magneticReset()
{
    writeReg(0x06, 0x07);
    delay(20);
}

/* ---- Timing control ---- */
unsigned long lastReadTime = 0;
const unsigned long READ_INTERVAL = 10;  // 100 Hz
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 100; // 10 Hz
 
 // Hard and soft iron calibration from calibration run
 const float HARD_IRON_X = 12.59f;
 const float HARD_IRON_Y = 4.65f;
 const float HARD_IRON_Z = -24.67f;
 
 const float SOFT_IRON_X = 0.747f;
 const float SOFT_IRON_Y = 0.875f;
 const float SOFT_IRON_Z = 1.926f;

unsigned long lastFusionTime = 0;
bool yawInitialized = false;
float fusedYawDeg = 0.0f;
bool yawWasRestoredFromNv = false;
bool magHeadingFilteredReady = false;
float magHeadingFilteredDeg = 0.0f;
bool outputHeadingReady = false;
float outputHeadingDeg = 0.0f;
bool lastImuHeadingReady = false;
float lastImuHeadingDeg = 0.0f;
float referenceFieldNorm_uT = 0.0f;
bool fieldReferenceReady = false;
bool magDisturbed = false;
float gyroZBiasDps = 0.0f;
bool gyroBiasReady = false;

const float MAG_CORRECTION_GAIN = 0.035f;
const float MAG_HEADING_LPF_ALPHA = 0.12f;
const float MAG_MAX_CORRECTION_DPS = 90.0f;
const float MAG_DISTURB_THRESHOLD_FRAC = 0.30f;
const float STATIONARY_ACCEL_TOL = 0.35f;
const float STATIONARY_GYRO_TOL_DPS = 1.0f;
const float GYRO_BIAS_ALPHA = 0.02f;

float clampf(float value, float minValue, float maxValue)
{
    if (value < minValue) return minValue;
    if (value > maxValue) return maxValue;
    return value;
}

float wrap360(float angle)
{
    while (angle < 0.0f) angle += 360.0f;
    while (angle >= 360.0f) angle -= 360.0f;
    return angle;
}

float wrap180(float angle)
{
    while (angle <= -180.0f) angle += 360.0f;
    while (angle > 180.0f) angle -= 360.0f;
    return angle;
}

void restorePersistentState()
{
    PersistedState state;
    if (!loadPersistedState(state))
        return;

    if (bnoReady && state.bnoOffsetsValid)
    {
        bno.setSensorOffsets(state.bnoOffsets);
        savedOffsetsAlready = true;
    }

    if (state.yawValid)
    {
        fusedYawDeg = wrap360(state.lastYawDeg);
        yawInitialized = true;
        yawWasRestoredFromNv = true;
        lastSavedYawDeg = fusedYawDeg;
    }
}

void checkpointPersistentState(unsigned long nowMs, bool forceSave)
{
    if (!forceSave && (nowMs - lastEepromSaveMs) < EEPROM_SAVE_INTERVAL_MS)
        return;

    PersistedState state = {};
    bool haveAnythingToSave = false;

    if (yawInitialized)
    {
        state.lastYawDeg = wrap360(fusedYawDeg);
        state.yawValid = 1;
        haveAnythingToSave = true;
    }

    if (bnoReady)
    {
        adafruit_bno055_offsets_t offsets;
        if (bno.getSensorOffsets(offsets))
        {
            state.bnoOffsets = offsets;
            state.bnoOffsetsValid = 1;
            haveAnythingToSave = true;
        }
    }

    if (!haveAnythingToSave)
        return;

    bool shouldSave = forceSave;

    if (state.yawValid)
    {
        float yawDeltaDeg = fabsf(wrap180(state.lastYawDeg - lastSavedYawDeg));
        if (yawDeltaDeg >= EEPROM_SAVE_MIN_DELTA_DEG)
            shouldSave = true;
    }

    if (state.bnoOffsetsValid && !savedOffsetsAlready && bnoReady && bno.isFullyCalibrated())
        shouldSave = true;

    if (!shouldSave)
        return;

    savePersistedState(state);
    lastEepromSaveMs = nowMs;

    if (state.yawValid)
        lastSavedYawDeg = state.lastYawDeg;
    if (state.bnoOffsetsValid)
        savedOffsetsAlready = true;
}

/* ---- Init state machine ---- */
enum InitState {
    INIT_WIRE_BEGIN,
    INIT_WAIT_WIRE,
    INIT_SOFT_RESET,
    INIT_WAIT_RESET,
    INIT_SET_NORMAL,
    INIT_WAIT_NORMAL,
    INIT_VERIFY_STATUS,
    INIT_DONE
};

InitState initState = INIT_WIRE_BEGIN;
unsigned long initTimer = 0;

void setup()
{
    Serial.begin(115200);

    bnoReady = bno.begin();
    if (bnoReady)
        bno.setExtCrystalUse(true);

    restorePersistentState();
    lastEepromSaveMs = millis();
}

void loop()
{
    unsigned long currentMillis = millis();

    if (initState != INIT_DONE)
    {
        switch (initState)
        {
        case INIT_WIRE_BEGIN:
            Wire.begin();
            initTimer = currentMillis;
            initState = INIT_WAIT_WIRE;
            break;

        case INIT_WAIT_WIRE:
            if (currentMillis - initTimer >= 100)
            {
                initState = INIT_SOFT_RESET;
            }
            break;

        case INIT_SOFT_RESET:
            writeReg(0x7E, 0xB6);
            initTimer = currentMillis;
            initState = INIT_WAIT_RESET;
            break;

        case INIT_WAIT_RESET:
            if (currentMillis - initTimer >= 50)
                initState = INIT_SET_NORMAL;
            break;

        case INIT_SET_NORMAL:
            writeReg(0x06, 0x01);
            writeReg(0x04, 0x04);
            initTimer = currentMillis;
            initState = INIT_WAIT_NORMAL;
            break;

        case INIT_WAIT_NORMAL:
            if (currentMillis - initTimer >= 10)
                initState = INIT_VERIFY_STATUS;
            break;

        case INIT_VERIFY_STATUS:
            magneticReset();
            lastReadTime = currentMillis;
            lastFusionTime = currentMillis;
            initState = INIT_DONE;
            break;

        default:
            break;
        }
        return;
    }

    if (currentMillis - lastReadTime >= READ_INTERVAL)
    {
        lastReadTime = currentMillis;

        uint8_t raw[9];

        if (readReg(0x31, raw, 9))
        {
            int32_t x = (int32_t)((raw[2] << 16) | (raw[1] << 8) | raw[0]);
            int32_t y = (int32_t)((raw[5] << 16) | (raw[4] << 8) | raw[3]);
            int32_t z = (int32_t)((raw[8] << 16) | (raw[7] << 8) | raw[6]);

            if (x & 0x800000) x |= 0xFF000000;
            if (y & 0x800000) y |= 0xFF000000;
            if (z & 0x800000) z |= 0xFF000000;

            const float SENSITIVITY = 300.0f;

            float x_uT = x / SENSITIVITY;
            float y_uT = y / SENSITIVITY;
            float z_uT = z / SENSITIVITY;

            imu::Vector<3> imuAcc;
            imu::Vector<3> imuGyro;
            bool imuSampleValid = false;

            if (bnoReady)
            {
                imuAcc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
                imuGyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
                imuSampleValid = true;

                uint8_t sysCal = 0, gyroCal = 0, accelCal = 0, magCal = 0;
                bno.getCalibration(&sysCal, &gyroCal, &accelCal, &magCal);
            }
            // apply hard/soft iron correction
            float x_corrected = (x_uT - HARD_IRON_X) * SOFT_IRON_X;
            float y_corrected = (y_uT - HARD_IRON_Y) * SOFT_IRON_Y;
            float z_corrected = (z_uT - HARD_IRON_Z) * SOFT_IRON_Z;

            float heading_mag = atan2f(y_corrected, x_corrected) * 180.0f / M_PI;
            heading_mag = wrap360(heading_mag);

            float heading_tilt = heading_mag;
            float heading_fused = heading_mag;

            if (imuSampleValid)
            {
                float accelX = imuAcc.x();
                float accelY = imuAcc.y();
                float accelZ = imuAcc.z();

                float roll = atan2f(accelY, accelZ);
                float pitch = atan2f(-accelX, sqrtf(accelY * accelY + accelZ * accelZ));

                float mx = x_corrected;
                float my = y_corrected;
                float mz = z_corrected;

                float mxh = (mx * cosf(pitch)) + (mz * sinf(pitch));
                float myh = (mx * sinf(roll) * sinf(pitch)) + (my * cosf(roll)) - (mz * sinf(roll) * cosf(pitch));

                heading_tilt = wrap360(atan2f(myh, mxh) * 180.0f / M_PI);

                if (!magHeadingFilteredReady)
                {
                    magHeadingFilteredDeg = heading_tilt;
                    magHeadingFilteredReady = true;
                }
                else
                {
                    float magDelta = wrap180(heading_tilt - magHeadingFilteredDeg);
                    magHeadingFilteredDeg = wrap360(magHeadingFilteredDeg + (MAG_HEADING_LPF_ALPHA * magDelta));
                }

                float dt = (currentMillis - lastFusionTime) * 0.001f;
                lastFusionTime = currentMillis;
                if (dt <= 0.0f || dt > 0.2f) dt = 0.01f;

                // BNO055 VECTOR_GYROSCOPE natively provides degrees per second
                float gyroZ_dps = imuGyro.z();
                float accelNorm = sqrtf((accelX * accelX) + (accelY * accelY) + (accelZ * accelZ));
                bool isStationary = (fabsf(accelNorm - 9.81f) < STATIONARY_ACCEL_TOL) &&
                                    (fabsf(gyroZ_dps) < STATIONARY_GYRO_TOL_DPS);

                if (isStationary)
                {
                    if (!gyroBiasReady)
                    {
                        gyroZBiasDps = gyroZ_dps;
                        gyroBiasReady = true;
                    }
                    else
                    {
                        gyroZBiasDps = ((1.0f - GYRO_BIAS_ALPHA) * gyroZBiasDps) + (GYRO_BIAS_ALPHA * gyroZ_dps);
                    }
                }

                float gyroZCorrected_dps = gyroZ_dps - gyroZBiasDps;

                if (!yawInitialized)
                {
                    fusedYawDeg = heading_tilt;
                    yawInitialized = true;
                    checkpointPersistentState(currentMillis, true);
                }

                fusedYawDeg = wrap360(fusedYawDeg + gyroZCorrected_dps * dt);

                float fieldNorm = sqrtf((mx * mx) + (my * my) + (mz * mz));
                if (!fieldReferenceReady)
                {
                    referenceFieldNorm_uT = fieldNorm;
                    fieldReferenceReady = true;
                }

                float normErrorFrac = fabsf(fieldNorm - referenceFieldNorm_uT) /
                                      (referenceFieldNorm_uT > 0.001f ? referenceFieldNorm_uT : 1.0f);
                magDisturbed = normErrorFrac > MAG_DISTURB_THRESHOLD_FRAC;

                if (!magDisturbed)
                {
                    referenceFieldNorm_uT = (0.98f * referenceFieldNorm_uT) + (0.02f * fieldNorm);
                    float yawError = wrap180(magHeadingFilteredDeg - fusedYawDeg);
                    float magCorrection = MAG_CORRECTION_GAIN * yawError;
                    float maxCorrectionStep = MAG_MAX_CORRECTION_DPS * dt;
                    magCorrection = clampf(magCorrection, -maxCorrectionStep, maxCorrectionStep);
                    fusedYawDeg = wrap360(fusedYawDeg + magCorrection);
                }

                heading_fused = fusedYawDeg;
            }

            if (currentMillis - lastPrintTime >= PRINT_INTERVAL)
            {
                lastPrintTime = currentMillis;

                float heading_output = heading_fused;

                Serial.print("BMM350 X:"); Serial.print(x_corrected, 2);
                Serial.print(" Y:"); Serial.print(y_corrected, 2);
                Serial.print(" Z:"); Serial.print(z_corrected, 2);
                Serial.println();

                Serial.print("BMM350 H:"); Serial.println(heading_output, 1);
            }
        }
    }

    checkpointPersistentState(currentMillis, false);
}




