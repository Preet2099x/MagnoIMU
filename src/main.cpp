#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stddef.h>
#include <string.h>
#include <EEPROM.h>

#define BMM350_ADDR 0x14
#define BMI088_ACCEL_ADDR 0x18
#define BMI088_GYRO_ADDR 0x68

#define BMI088_CHIP_ID_ADDR 0x00
#define BMI088_ACCEL_CHIP_ID 0x1E
#define BMI088_GYRO_CHIP_ID 0x0F

#define BMI088_ACCEL_DATA_X_LSB_ADDR 0x12
#define BMI088_GYRO_DATA_X_LSB_ADDR 0x02

#define BMI088_ACCEL_SOFTRESET_ADDR 0x7E
#define BMI088_ACCEL_SOFTRESET_CMD 0xB6
#define BMI088_ACCEL_PWR_CONF_ADDR 0x7C
#define BMI088_ACCEL_PWR_CTRL_ADDR 0x7D
#define BMI088_ACCEL_CONF_ADDR 0x40
#define BMI088_ACCEL_RANGE_ADDR 0x41

#define BMI088_GYRO_SOFTRESET_ADDR 0x14
#define BMI088_GYRO_SOFTRESET_CMD 0xB6
#define BMI088_GYRO_RANGE_ADDR 0x0F
#define BMI088_GYRO_BW_ADDR 0x10
#define BMI088_GYRO_LPM1_ADDR 0x11

bool bnoReady = false;
uint8_t bnoMode = 0;
uint8_t bmiAccelAddr = BMI088_ACCEL_ADDR;
uint8_t bmiGyroAddr = BMI088_GYRO_ADDR;

bool writeBmiReg(uint8_t addr, uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(value);
    return (Wire.endTransmission() == 0);
}

bool readBmiReg(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len)
{
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0)
        return false;

    if (Wire.requestFrom(static_cast<uint8_t>(addr), len) != len)
        return false;

    for (uint8_t i = 0; i < len; i++)
        data[i] = Wire.read();

    return true;
}

bool setBnoOperationMode(uint8_t mode)
{
    bnoMode = mode;
    delay(1);
    return true;
}

bool bnoBegin()
{
    const uint8_t accelAddrCandidates[] = {0x18, 0x19};
    const uint8_t gyroAddrCandidates[] = {0x68, 0x69};
    uint8_t accelChipId = 0;
    uint8_t gyroChipId = 0;
    bool accelFound = false;
    bool gyroFound = false;

    for (uint8_t i = 0; i < sizeof(accelAddrCandidates); i++)
    {
        if (readBmiReg(accelAddrCandidates[i], BMI088_CHIP_ID_ADDR, &accelChipId, 1) &&
            (accelChipId == BMI088_ACCEL_CHIP_ID || accelChipId == 0x1F))
        {
            bmiAccelAddr = accelAddrCandidates[i];
            accelFound = true;
            break;
        }
    }

    for (uint8_t i = 0; i < sizeof(gyroAddrCandidates); i++)
    {
        if (readBmiReg(gyroAddrCandidates[i], BMI088_CHIP_ID_ADDR, &gyroChipId, 1) && gyroChipId == BMI088_GYRO_CHIP_ID)
        {
            bmiGyroAddr = gyroAddrCandidates[i];
            gyroFound = true;
            break;
        }
    }

    if (!accelFound)
        return false;

    if (!setBnoOperationMode(0x00))
        return false;

    if (!writeBmiReg(bmiAccelAddr, BMI088_ACCEL_SOFTRESET_ADDR, BMI088_ACCEL_SOFTRESET_CMD))
        return false;
    delay(50);

    if (gyroFound)
    {
        if (!writeBmiReg(bmiGyroAddr, BMI088_GYRO_SOFTRESET_ADDR, BMI088_GYRO_SOFTRESET_CMD))
            gyroFound = false;
        delay(50);
    }

    if (!writeBmiReg(bmiAccelAddr, BMI088_ACCEL_PWR_CONF_ADDR, 0x00))
        return false;
    delay(5);

    if (!writeBmiReg(bmiAccelAddr, BMI088_ACCEL_PWR_CTRL_ADDR, 0x04))
        return false;
    delay(5);

    if (!writeBmiReg(bmiAccelAddr, BMI088_ACCEL_CONF_ADDR, 0xA8))
        return false;

    if (!writeBmiReg(bmiAccelAddr, BMI088_ACCEL_RANGE_ADDR, 0x01))
        return false;

    if (gyroFound)
    {
        if (!writeBmiReg(bmiGyroAddr, BMI088_GYRO_RANGE_ADDR, 0x04) ||
            !writeBmiReg(bmiGyroAddr, BMI088_GYRO_BW_ADDR, 0x07) ||
            !writeBmiReg(bmiGyroAddr, BMI088_GYRO_LPM1_ADDR, 0x00))
        {
            gyroFound = false;
        }
        delay(30);
    }

    if (!setBnoOperationMode(0x01))
        return false;

    return true;
}

int16_t decodeBnoInt16(uint8_t lsb, uint8_t msb)
{
    return static_cast<int16_t>((static_cast<int16_t>(msb) << 8) | lsb);
}

bool readBnoRawAccelGyro(float &accelX_mps2,
                         float &accelY_mps2,
                         float &accelZ_mps2,
                         float &gyroX_dps,
                         float &gyroY_dps,
                         float &gyroZ_dps)
{
    uint8_t accelRaw[6];
    uint8_t gyroRaw[6];

    if (!readBmiReg(bmiAccelAddr, BMI088_ACCEL_DATA_X_LSB_ADDR, accelRaw, sizeof(accelRaw)))
        return false;

    bool gyroOk = readBmiReg(bmiGyroAddr, BMI088_GYRO_DATA_X_LSB_ADDR, gyroRaw, sizeof(gyroRaw));

    int16_t accX = decodeBnoInt16(accelRaw[0], accelRaw[1]);
    int16_t accY = decodeBnoInt16(accelRaw[2], accelRaw[3]);
    int16_t accZ = decodeBnoInt16(accelRaw[4], accelRaw[5]);

    int16_t gyrX = gyroOk ? decodeBnoInt16(gyroRaw[0], gyroRaw[1]) : 0;
    int16_t gyrY = gyroOk ? decodeBnoInt16(gyroRaw[2], gyroRaw[3]) : 0;
    int16_t gyrZ = gyroOk ? decodeBnoInt16(gyroRaw[4], gyroRaw[5]) : 0;

    const float BMI088_ACCEL_LSB_PER_G = 5460.0f;
    const float BMI088_G = 9.80665f;
    const float BMI088_GYRO_LSB_PER_DPS = 262.4f;

    accelX_mps2 = (static_cast<float>(accX) / BMI088_ACCEL_LSB_PER_G) * BMI088_G;
    accelY_mps2 = (static_cast<float>(accY) / BMI088_ACCEL_LSB_PER_G) * BMI088_G;
    accelZ_mps2 = (static_cast<float>(accZ) / BMI088_ACCEL_LSB_PER_G) * BMI088_G;

    gyroX_dps = static_cast<float>(gyrX) / BMI088_GYRO_LSB_PER_DPS;
    gyroY_dps = static_cast<float>(gyrY) / BMI088_GYRO_LSB_PER_DPS;
    gyroZ_dps = static_cast<float>(gyrZ) / BMI088_GYRO_LSB_PER_DPS;

    return true;
}

// Persisted state stored in EEPROM (keeps last yaw)
struct PersistedState
{
    uint32_t magic;
    uint16_t version;
    uint16_t checksum;
    float lastYawDeg;
    uint8_t yawValid;
    uint8_t reserved[3];
};

const uint32_t PERSIST_MAGIC = 0x4D494D55UL;
const uint16_t PERSIST_VERSION = 2;
const int EEPROM_ADDR = 0;
const unsigned long EEPROM_SAVE_INTERVAL_MS = 2000;
const float EEPROM_SAVE_MIN_DELTA_DEG = 0.5f;

unsigned long lastEepromSaveMs = 0;
float lastSavedYawDeg = 0.0f;

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

// BMM350 I2C read (2 dummy bytes)
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

// BMM350 I2C write
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

unsigned long lastReadTime = 0;
const unsigned long READ_INTERVAL = 10;
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 100; // 10 Hz
 
// Hard/soft iron calibration constants
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
float referenceFieldNorm_uT = 0.0f;
bool fieldReferenceReady = false;
bool magDisturbed = false;
bool ekfInitialized = false;
float ekfYawRad = 0.0f;
float ekfGyroBiasRad = 0.0f;
float ekfP00 = 0.35f;
float ekfP01 = 0.0f;
float ekfP10 = 0.0f;
float ekfP11 = 0.05f;

const float MAG_DISTURB_THRESHOLD_FRAC = 0.30f;
const float STATIONARY_ACCEL_TOL = 0.35f;
const float STATIONARY_GYRO_TOL_DPS = 1.0f;
const float GYRO_RATE_CLAMP_DPS = 250.0f;
const float EKF_GYRO_NOISE_STD_RADPS = 0.25f;
const float EKF_BIAS_RW_STD_RADPS2 = 0.002f;
const float EKF_MAG_MEAS_STD_RAD = 0.08f;
const float EKF_MAG_MEAS_STD_RAD_STILL = 0.015f;
const float EKF_ZERO_RATE_STD_RADPS = 0.02f;
const float EKF_MIN_P00 = 1.0e-5f;
const float EKF_MAX_P11 = 0.25f;
const float MAG_MIN_HORIZONTAL_FIELD_uT = 5.0f;
const float EKF_MAG_INNOVATION_GATE_SIGMA = 3.0f;

const float DEG2RAD = 0.01745329251994329576923690768489f;
const float RAD2DEG = 57.295779513082320876798154814105f;

// Math utilities
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

float wrapPi(float angle)
{
    while (angle <= -M_PI) angle += 2.0f * M_PI;
    while (angle > M_PI) angle -= 2.0f * M_PI;
    return angle;
}

// Extended Kalman Filter (1D yaw) helpers
void initializeEkfYaw(float yawRadSeed)
{
    ekfYawRad = wrapPi(yawRadSeed);
    ekfGyroBiasRad = 0.0f;
    ekfP00 = 0.35f;
    ekfP01 = 0.0f;
    ekfP10 = 0.0f;
    ekfP11 = 0.05f;
    ekfInitialized = true;
}

void ekfPredict(float dt, float gyroZRad)
{
    float qYaw = EKF_GYRO_NOISE_STD_RADPS * EKF_GYRO_NOISE_STD_RADPS * dt;
    float qBias = EKF_BIAS_RW_STD_RADPS2 * EKF_BIAS_RW_STD_RADPS2 * dt;

    ekfYawRad = wrapPi(ekfYawRad + (gyroZRad - ekfGyroBiasRad) * dt);

    float oldP00 = ekfP00;
    float oldP01 = ekfP01;
    float oldP10 = ekfP10;
    float oldP11 = ekfP11;

    ekfP00 = oldP00 - dt * oldP10 - dt * oldP01 + dt * dt * oldP11 + qYaw;
    ekfP01 = oldP01 - dt * oldP11;
    ekfP10 = oldP10 - dt * oldP11;
    ekfP11 = oldP11 + qBias;

    if (ekfP00 < EKF_MIN_P00)
        ekfP00 = EKF_MIN_P00;
    if (ekfP11 > EKF_MAX_P11)
        ekfP11 = EKF_MAX_P11;
}

void ekfUpdateFromMagYaw(float magYawRad, float measurementStdRad)
{
    float r = measurementStdRad * measurementStdRad;
    float innovation = wrapPi(magYawRad - ekfYawRad);
    float s = ekfP00 + r;
    if (s <= 1e-9f)
        return;

    float k0 = ekfP00 / s;
    float k1 = ekfP10 / s;

    ekfYawRad = wrapPi(ekfYawRad + k0 * innovation);
    ekfGyroBiasRad += k1 * innovation;

    float oldP00 = ekfP00;
    float oldP01 = ekfP01;
    float oldP10 = ekfP10;
    float oldP11 = ekfP11;

    ekfP00 = oldP00 - k0 * oldP00;
    ekfP01 = oldP01 - k0 * oldP01;
    ekfP10 = oldP10 - k1 * oldP00;
    ekfP11 = oldP11 - k1 * oldP01;

    float pSym = 0.5f * (ekfP01 + ekfP10);
    ekfP01 = pSym;
    ekfP10 = pSym;

    if (ekfP00 < EKF_MIN_P00)
        ekfP00 = EKF_MIN_P00;
    if (ekfP11 < 1.0e-7f)
        ekfP11 = 1.0e-7f;
}

bool shouldUseMagYaw(float magYawRad,
                     float measurementStdRad,
                     bool magneticEnvironmentHealthy,
                     float horizontalField_uT)
{
    if (!magneticEnvironmentHealthy)
        return false;

    if (horizontalField_uT < MAG_MIN_HORIZONTAL_FIELD_uT)
        return false;

    float innovation = wrapPi(magYawRad - ekfYawRad);
    float r = measurementStdRad * measurementStdRad;
    float s = ekfP00 + r;
    if (s <= 1e-9f)
        return false;

    float gate = EKF_MAG_INNOVATION_GATE_SIGMA * sqrtf(s);
    return fabsf(innovation) <= gate;
}

void ekfUpdateZeroRate(float gyroZRad, float zeroRateStdRad)
{
    float r = zeroRateStdRad * zeroRateStdRad;
    float innovation = gyroZRad - ekfGyroBiasRad;
    float s = ekfP11 + r;
    if (s <= 1e-12f)
        return;

    float k0 = ekfP01 / s;
    float k1 = ekfP11 / s;

    ekfYawRad = wrapPi(ekfYawRad + k0 * innovation);
    ekfGyroBiasRad += k1 * innovation;

    float oldP00 = ekfP00;
    float oldP01 = ekfP01;
    float oldP10 = ekfP10;
    float oldP11 = ekfP11;

    ekfP00 = oldP00 - k0 * oldP10;
    ekfP01 = oldP01 - k0 * oldP11;
    ekfP10 = oldP10 - k1 * oldP10;
    ekfP11 = oldP11 - k1 * oldP11;

    float pSym = 0.5f * (ekfP01 + ekfP10);
    ekfP01 = pSym;
    ekfP10 = pSym;

    if (ekfP00 < EKF_MIN_P00)
        ekfP00 = EKF_MIN_P00;
    if (ekfP11 < 1.0e-7f)
        ekfP11 = 1.0e-7f;
}

// Persistent state helpers
void restorePersistentState()
{
    PersistedState state;
    if (!loadPersistedState(state))
        return;

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

    if (!yawInitialized)
        return;

    PersistedState state = {};
    state.lastYawDeg = wrap360(fusedYawDeg);
    state.yawValid = 1;

    bool shouldSave = forceSave;

    float yawDeltaDeg = fabsf(wrap180(state.lastYawDeg - lastSavedYawDeg));
    if (yawDeltaDeg >= EEPROM_SAVE_MIN_DELTA_DEG)
        shouldSave = true;

    if (!shouldSave)
        return;

    savePersistedState(state);
    lastEepromSaveMs = nowMs;
    lastSavedYawDeg = state.lastYawDeg;
}

// Init state machine
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

    Wire.begin();

    bnoReady = bnoBegin();

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

            bool imuSampleValid = false;
            float accelX = 0.0f;
            float accelY = 0.0f;
            float accelZ = 0.0f;
            float gyroX_dps = 0.0f;
            float gyroY_dps = 0.0f;
            float gyroZ_dps = 0.0f;
            float rollDeg = 0.0f;
            float pitchDeg = 0.0f;

            if (bnoReady)
            {
                imuSampleValid = readBnoRawAccelGyro(accelX,
                                                     accelY,
                                                     accelZ,
                                                     gyroX_dps,
                                                     gyroY_dps,
                                                     gyroZ_dps);
            }

            float x_corrected = (x_uT - HARD_IRON_X) * SOFT_IRON_X;
            float y_corrected = (y_uT - HARD_IRON_Y) * SOFT_IRON_Y;
            float z_corrected = (z_uT - HARD_IRON_Z) * SOFT_IRON_Z;

            float heading_mag = atan2f(y_corrected, x_corrected) * 180.0f / M_PI;
            heading_mag = wrap360(heading_mag);

            float heading_fused = yawInitialized ? fusedYawDeg : heading_mag;

            if (imuSampleValid)
            {
                float dt = (currentMillis - lastFusionTime) * 0.001f;
                lastFusionTime = currentMillis;
                if (dt <= 0.0f || dt > 0.2f) dt = 0.01f;

                float mx = x_corrected;
                float my = y_corrected;
                float mz = z_corrected;

                float roll = atan2f(accelY, accelZ);
                float pitch = atan2f(-accelX, sqrtf((accelY * accelY) + (accelZ * accelZ)));
                rollDeg = roll * RAD2DEG;
                pitchDeg = pitch * RAD2DEG;
                float mxh = (mx * cosf(pitch)) + (mz * sinf(pitch));
                float myh = (mx * sinf(roll) * sinf(pitch)) + (my * cosf(roll)) - (mz * sinf(roll) * cosf(pitch));
                float heading_tilt = wrap360(atan2f(myh, mxh) * 180.0f / M_PI);

                gyroX_dps = clampf(gyroX_dps, -GYRO_RATE_CLAMP_DPS, GYRO_RATE_CLAMP_DPS);
                gyroY_dps = clampf(gyroY_dps, -GYRO_RATE_CLAMP_DPS, GYRO_RATE_CLAMP_DPS);
                gyroZ_dps = clampf(gyroZ_dps, -GYRO_RATE_CLAMP_DPS, GYRO_RATE_CLAMP_DPS);

                float accelNorm = sqrtf((accelX * accelX) + (accelY * accelY) + (accelZ * accelZ));
                float gyroNormDps = sqrtf((gyroX_dps * gyroX_dps) + (gyroY_dps * gyroY_dps) + (gyroZ_dps * gyroZ_dps));
                bool isStationary = (fabsf(accelNorm - 9.81f) < STATIONARY_ACCEL_TOL) &&
                                    (gyroNormDps < STATIONARY_GYRO_TOL_DPS);

                float gyroZForEkf_dps = gyroZ_dps;

                float fieldNorm = sqrtf((mx * mx) + (my * my) + (mz * mz));
                if (!fieldReferenceReady)
                {
                    referenceFieldNorm_uT = fieldNorm;
                    fieldReferenceReady = true;
                }

                float normErrorFrac = fabsf(fieldNorm - referenceFieldNorm_uT) /
                                      (referenceFieldNorm_uT > 0.001f ? referenceFieldNorm_uT : 1.0f);
                magDisturbed = normErrorFrac > MAG_DISTURB_THRESHOLD_FRAC;

                if (!ekfInitialized)
                {
                    float yawSeedRad = (yawWasRestoredFromNv && yawInitialized)
                                           ? wrap360(fusedYawDeg) * DEG2RAD
                                           : heading_tilt * DEG2RAD;
                    initializeEkfYaw(yawSeedRad);

                    if (!yawInitialized)
                    {
                        fusedYawDeg = wrap360(ekfYawRad * RAD2DEG);
                        yawInitialized = true;
                        checkpointPersistentState(currentMillis, true);
                    }
                }

                ekfPredict(dt, gyroZForEkf_dps * DEG2RAD);

                if (isStationary)
                {
                    ekfUpdateZeroRate(gyroZForEkf_dps * DEG2RAD, EKF_ZERO_RATE_STD_RADPS);
                }

                float measStd = isStationary ? EKF_MAG_MEAS_STD_RAD_STILL : EKF_MAG_MEAS_STD_RAD;
                bool magYawValid = shouldUseMagYaw(heading_tilt * DEG2RAD,
                                                   measStd,
                                                   !magDisturbed,
                                                   sqrtf((mxh * mxh) + (myh * myh)));
                if (magYawValid)
                {
                    referenceFieldNorm_uT = (0.98f * referenceFieldNorm_uT) + (0.02f * fieldNorm);
                    ekfUpdateFromMagYaw(heading_tilt * DEG2RAD, measStd);
                }

                fusedYawDeg = wrap360(ekfYawRad * RAD2DEG);
                heading_fused = fusedYawDeg;
            }

            if (currentMillis - lastPrintTime >= PRINT_INTERVAL)
            {
                lastPrintTime = currentMillis;

                float heading_output = imuSampleValid
                                           ? (yawInitialized ? fusedYawDeg : heading_fused)
                                           : heading_mag;

                if (imuSampleValid)
                {
                    Serial.print("IMU Roll:"); Serial.print(rollDeg, 1);
                    Serial.print(" Pitch:"); Serial.println(pitchDeg, 1);
                }
                else
                {
                    Serial.println("IMU Roll:N/A Pitch:N/A");
                }

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