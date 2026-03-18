#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stddef.h>
#include <string.h>
#include <EEPROM.h>

#define BMM350_ADDR 0x14
#define BNO055_ADDR 0x28

#define BNO055_CHIP_ID_ADDR 0x00
#define BNO055_PAGE_ID_ADDR 0x07
#define BNO055_ACCEL_DATA_X_LSB_ADDR 0x08
#define BNO055_GYRO_DATA_X_LSB_ADDR 0x14
#define BNO055_CALIB_STAT_ADDR 0x35
#define BNO055_UNIT_SEL_ADDR 0x3B
#define BNO055_OPR_MODE_ADDR 0x3D
#define BNO055_PWR_MODE_ADDR 0x3E
#define BNO055_SYS_TRIGGER_ADDR 0x3F
#define BNO055_ACC_OFFSET_X_LSB_ADDR 0x55

#define BNO055_CHIP_ID 0xA0
#define BNO055_OPERATION_MODE_CONFIG 0x00
#define BNO055_OPERATION_MODE_NDOF 0x0C
#define BNO055_POWER_MODE_NORMAL 0x00
#define BNO055_OFFSETS_LEN 22

bool bnoReady = false;
uint8_t bnoMode = BNO055_OPERATION_MODE_CONFIG;

bool writeBnoReg(uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(BNO055_ADDR);
    Wire.write(reg);
    Wire.write(value);
    return (Wire.endTransmission() == 0);
}

bool readBnoReg(uint8_t reg, uint8_t *data, uint8_t len)
{
    Wire.beginTransmission(BNO055_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0)
        return false;

    if (Wire.requestFrom(static_cast<uint8_t>(BNO055_ADDR), len) != len)
        return false;

    for (uint8_t i = 0; i < len; i++)
        data[i] = Wire.read();

    return true;
}

bool setBnoOperationMode(uint8_t mode)
{
    if (!writeBnoReg(BNO055_OPR_MODE_ADDR, mode))
        return false;
    bnoMode = mode;
    delay(30);
    return true;
}

bool bnoSetExtCrystalUse(bool useExternalCrystal)
{
    uint8_t modeToRestore = bnoMode;
    if (!setBnoOperationMode(BNO055_OPERATION_MODE_CONFIG))
        return false;

    if (!writeBnoReg(BNO055_PAGE_ID_ADDR, 0x00))
        return false;

    if (!writeBnoReg(BNO055_SYS_TRIGGER_ADDR, useExternalCrystal ? 0x80 : 0x00))
        return false;

    delay(10);
    return setBnoOperationMode(modeToRestore);
}

bool bnoBegin()
{
    uint8_t chipId = 0;
    if (!readBnoReg(BNO055_CHIP_ID_ADDR, &chipId, 1) || chipId != BNO055_CHIP_ID)
    {
        delay(650);
        if (!readBnoReg(BNO055_CHIP_ID_ADDR, &chipId, 1) || chipId != BNO055_CHIP_ID)
            return false;
    }

    if (!setBnoOperationMode(BNO055_OPERATION_MODE_CONFIG))
        return false;

    if (!writeBnoReg(BNO055_PAGE_ID_ADDR, 0x00))
        return false;

    if (!writeBnoReg(BNO055_SYS_TRIGGER_ADDR, 0x20))
        return false;

    delay(650);

    if (!readBnoReg(BNO055_CHIP_ID_ADDR, &chipId, 1) || chipId != BNO055_CHIP_ID)
        return false;

    if (!writeBnoReg(BNO055_PWR_MODE_ADDR, BNO055_POWER_MODE_NORMAL))
        return false;

    delay(10);

    if (!writeBnoReg(BNO055_PAGE_ID_ADDR, 0x00))
        return false;

    if (!writeBnoReg(BNO055_UNIT_SEL_ADDR, 0x00))
        return false;

    if (!writeBnoReg(BNO055_SYS_TRIGGER_ADDR, 0x00))
        return false;

    delay(10);

    if (!setBnoOperationMode(BNO055_OPERATION_MODE_NDOF))
        return false;

    return true;
}

bool bnoGetSensorOffsets(uint8_t *offsets)
{
    uint8_t modeToRestore = bnoMode;
    if (!setBnoOperationMode(BNO055_OPERATION_MODE_CONFIG))
        return false;

    if (!writeBnoReg(BNO055_PAGE_ID_ADDR, 0x00))
        return false;

    bool ok = readBnoReg(BNO055_ACC_OFFSET_X_LSB_ADDR, offsets, BNO055_OFFSETS_LEN);
    if (!setBnoOperationMode(modeToRestore))
        return false;

    return ok;
}

bool bnoSetSensorOffsets(const uint8_t *offsets)
{
    uint8_t modeToRestore = bnoMode;
    if (!setBnoOperationMode(BNO055_OPERATION_MODE_CONFIG))
        return false;

    if (!writeBnoReg(BNO055_PAGE_ID_ADDR, 0x00))
        return false;

    for (uint8_t i = 0; i < BNO055_OFFSETS_LEN; i++)
    {
        if (!writeBnoReg(static_cast<uint8_t>(BNO055_ACC_OFFSET_X_LSB_ADDR + i), offsets[i]))
            return false;
    }

    return setBnoOperationMode(modeToRestore);
}

bool bnoIsFullyCalibrated()
{
    uint8_t calib = 0;
    if (!readBnoReg(BNO055_CALIB_STAT_ADDR, &calib, 1))
        return false;

    uint8_t sys = (calib >> 6) & 0x03;
    uint8_t gyro = (calib >> 4) & 0x03;
    uint8_t accel = (calib >> 2) & 0x03;
    uint8_t mag = calib & 0x03;

    return (sys == 3) && (gyro == 3) && (accel == 3) && (mag == 3);
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

    if (!readBnoReg(BNO055_ACCEL_DATA_X_LSB_ADDR, accelRaw, sizeof(accelRaw)))
        return false;
    if (!readBnoReg(BNO055_GYRO_DATA_X_LSB_ADDR, gyroRaw, sizeof(gyroRaw)))
        return false;

    int16_t accX = decodeBnoInt16(accelRaw[0], accelRaw[1]);
    int16_t accY = decodeBnoInt16(accelRaw[2], accelRaw[3]);
    int16_t accZ = decodeBnoInt16(accelRaw[4], accelRaw[5]);

    int16_t gyrX = decodeBnoInt16(gyroRaw[0], gyroRaw[1]);
    int16_t gyrY = decodeBnoInt16(gyroRaw[2], gyroRaw[3]);
    int16_t gyrZ = decodeBnoInt16(gyroRaw[4], gyroRaw[5]);

    accelX_mps2 = static_cast<float>(accX) / 100.0f;
    accelY_mps2 = static_cast<float>(accY) / 100.0f;
    accelZ_mps2 = static_cast<float>(accZ) / 100.0f;

    gyroX_dps = static_cast<float>(gyrX) / 16.0f;
    gyroY_dps = static_cast<float>(gyrY) / 16.0f;
    gyroZ_dps = static_cast<float>(gyrZ) / 16.0f;

    return true;
}

struct PersistedState
{
    uint32_t magic;
    uint16_t version;
    uint16_t checksum;
    float lastYawDeg;
    uint8_t yawValid;
    uint8_t bnoOffsetsValid;
    uint8_t reserved[2];
    uint8_t bnoOffsets[BNO055_OFFSETS_LEN];
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
float referenceFieldNorm_uT = 0.0f;
bool fieldReferenceReady = false;
bool magDisturbed = false;
bool ahrsInitialized = false;
float q0 = 1.0f;
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;
float gyroBiasXDps = 0.0f;
float gyroBiasYDps = 0.0f;
float gyroBiasZDps = 0.0f;
bool gyroBiasReady = false;

const float MAG_DISTURB_THRESHOLD_FRAC = 0.30f;
const float STATIONARY_ACCEL_TOL = 0.35f;
const float STATIONARY_GYRO_TOL_DPS = 1.0f;
const float GYRO_BIAS_ALPHA = 0.02f;
const float GYRO_RATE_CLAMP_DPS = 250.0f;
const float MAHONY_KP = 2.2f;
const float MAHONY_KI = 0.02f;
const float MAHONY_MAG_WEIGHT = 1.0f;

float integralFBx = 0.0f;
float integralFBy = 0.0f;
float integralFBz = 0.0f;

const float DEG2RAD = 0.01745329251994329576923690768489f;
const float RAD2DEG = 57.295779513082320876798154814105f;

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

float invSqrt(float value)
{
    if (value <= 0.0f)
        return 0.0f;
    return 1.0f / sqrtf(value);
}

void setQuaternionFromEuler(float rollRad, float pitchRad, float yawRad)
{
    float cr = cosf(rollRad * 0.5f);
    float sr = sinf(rollRad * 0.5f);
    float cp = cosf(pitchRad * 0.5f);
    float sp = sinf(pitchRad * 0.5f);
    float cy = cosf(yawRad * 0.5f);
    float sy = sinf(yawRad * 0.5f);

    q0 = (cr * cp * cy) + (sr * sp * sy);
    q1 = (sr * cp * cy) - (cr * sp * sy);
    q2 = (cr * sp * cy) + (sr * cp * sy);
    q3 = (cr * cp * sy) - (sr * sp * cy);

    float recipNorm = invSqrt((q0 * q0) + (q1 * q1) + (q2 * q2) + (q3 * q3));
    if (recipNorm > 0.0f)
    {
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;
    }
}

float quaternionYawDeg()
{
    float yaw = atan2f(2.0f * ((q0 * q3) + (q1 * q2)),
                       1.0f - (2.0f * ((q2 * q2) + (q3 * q3))));
    return wrap360(yaw * RAD2DEG);
}

void initializeAhrsPose(float accelX, float accelY, float accelZ,
                        float magX, float magY, float magZ,
                        float yawSeedDeg, bool useYawSeed)
{
    float roll = atan2f(accelY, accelZ);
    float pitch = atan2f(-accelX, sqrtf((accelY * accelY) + (accelZ * accelZ)));

    float mxh = (magX * cosf(pitch)) + (magZ * sinf(pitch));
    float myh = (magX * sinf(roll) * sinf(pitch)) + (magY * cosf(roll)) - (magZ * sinf(roll) * cosf(pitch));

    float yaw = atan2f(myh, mxh);
    if (useYawSeed)
        yaw = wrap360(yawSeedDeg) * DEG2RAD;

    setQuaternionFromEuler(roll, pitch, yaw);
    ahrsInitialized = true;
}

void updateAhrsMahony(float dt,
                      float gxRad,
                      float gyRad,
                      float gzRad,
                      float ax,
                      float ay,
                      float az,
                      float mx,
                      float my,
                      float mz,
                      float magWeight)
{
    float recipNorm = invSqrt((ax * ax) + (ay * ay) + (az * az));
    if (recipNorm <= 0.0f)
        return;

    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    float vx = 2.0f * ((q1 * q3) - (q0 * q2));
    float vy = 2.0f * ((q0 * q1) + (q2 * q3));
    float vz = (q0 * q0) - (q1 * q1) - (q2 * q2) + (q3 * q3);

    float ex = (ay * vz) - (az * vy);
    float ey = (az * vx) - (ax * vz);
    float ez = (ax * vy) - (ay * vx);

    if (magWeight > 0.0f)
    {
        recipNorm = invSqrt((mx * mx) + (my * my) + (mz * mz));
        if (recipNorm > 0.0f)
        {
            mx *= recipNorm;
            my *= recipNorm;
            mz *= recipNorm;

            float q0q1 = q0 * q1;
            float q0q2 = q0 * q2;
            float q0q3 = q0 * q3;
            float q1q1 = q1 * q1;
            float q1q2 = q1 * q2;
            float q1q3 = q1 * q3;
            float q2q2 = q2 * q2;
            float q2q3 = q2 * q3;
            float q3q3 = q3 * q3;

            float hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
            float hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
            float bx = sqrtf((hx * hx) + (hy * hy));
            float bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

            float wx = 2.0f * (bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2));
            float wy = 2.0f * (bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3));
            float wz = 2.0f * (bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2));

            float mex = (my * wz) - (mz * wy);
            float mey = (mz * wx) - (mx * wz);
            float mez = (mx * wy) - (my * wx);

            ex += mex * magWeight;
            ey += mey * magWeight;
            ez += mez * magWeight;
        }
    }

    integralFBx += (MAHONY_KI * ex * dt);
    integralFBy += (MAHONY_KI * ey * dt);
    integralFBz += (MAHONY_KI * ez * dt);

    gxRad += (MAHONY_KP * ex) + integralFBx;
    gyRad += (MAHONY_KP * ey) + integralFBy;
    gzRad += (MAHONY_KP * ez) + integralFBz;

    float qDot0 = 0.5f * ((-q1 * gxRad) - (q2 * gyRad) - (q3 * gzRad));
    float qDot1 = 0.5f * ((q0 * gxRad) + (q2 * gzRad) - (q3 * gyRad));
    float qDot2 = 0.5f * ((q0 * gyRad) - (q1 * gzRad) + (q3 * gxRad));
    float qDot3 = 0.5f * ((q0 * gzRad) + (q1 * gyRad) - (q2 * gxRad));

    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    recipNorm = invSqrt((q0 * q0) + (q1 * q1) + (q2 * q2) + (q3 * q3));
    if (recipNorm > 0.0f)
    {
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;
    }
}

void restorePersistentState()
{
    PersistedState state;
    if (!loadPersistedState(state))
        return;

    if (bnoReady && state.bnoOffsetsValid)
    {
        if (bnoSetSensorOffsets(state.bnoOffsets))
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
        uint8_t offsets[BNO055_OFFSETS_LEN];
        if (bnoGetSensorOffsets(offsets))
        {
            memcpy(state.bnoOffsets, offsets, BNO055_OFFSETS_LEN);
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

    if (state.bnoOffsetsValid && !savedOffsetsAlready && bnoReady && bnoIsFullyCalibrated())
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

    Wire.begin();

    bnoReady = bnoBegin();
    if (bnoReady)
        bnoSetExtCrystalUse(true);

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

            if (bnoReady)
            {
                imuSampleValid = readBnoRawAccelGyro(accelX,
                                                     accelY,
                                                     accelZ,
                                                     gyroX_dps,
                                                     gyroY_dps,
                                                     gyroZ_dps);
            }

            // apply hard/soft iron correction
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

                gyroX_dps = clampf(gyroX_dps, -GYRO_RATE_CLAMP_DPS, GYRO_RATE_CLAMP_DPS);
                gyroY_dps = clampf(gyroY_dps, -GYRO_RATE_CLAMP_DPS, GYRO_RATE_CLAMP_DPS);
                gyroZ_dps = clampf(gyroZ_dps, -GYRO_RATE_CLAMP_DPS, GYRO_RATE_CLAMP_DPS);

                float accelNorm = sqrtf((accelX * accelX) + (accelY * accelY) + (accelZ * accelZ));
                float gyroNormDps = sqrtf((gyroX_dps * gyroX_dps) + (gyroY_dps * gyroY_dps) + (gyroZ_dps * gyroZ_dps));
                bool isStationary = (fabsf(accelNorm - 9.81f) < STATIONARY_ACCEL_TOL) &&
                                    (gyroNormDps < STATIONARY_GYRO_TOL_DPS);

                if (isStationary)
                {
                    if (!gyroBiasReady)
                    {
                        gyroBiasXDps = gyroX_dps;
                        gyroBiasYDps = gyroY_dps;
                        gyroBiasZDps = gyroZ_dps;
                        gyroBiasReady = true;
                    }
                    else
                    {
                        gyroBiasXDps = ((1.0f - GYRO_BIAS_ALPHA) * gyroBiasXDps) + (GYRO_BIAS_ALPHA * gyroX_dps);
                        gyroBiasYDps = ((1.0f - GYRO_BIAS_ALPHA) * gyroBiasYDps) + (GYRO_BIAS_ALPHA * gyroY_dps);
                        gyroBiasZDps = ((1.0f - GYRO_BIAS_ALPHA) * gyroBiasZDps) + (GYRO_BIAS_ALPHA * gyroZ_dps);
                    }
                }

                float gyroXCorrected_dps = gyroX_dps - gyroBiasXDps;
                float gyroYCorrected_dps = gyroY_dps - gyroBiasYDps;
                float gyroZCorrected_dps = gyroZ_dps - gyroBiasZDps;

                float fieldNorm = sqrtf((mx * mx) + (my * my) + (mz * mz));
                if (!fieldReferenceReady)
                {
                    referenceFieldNorm_uT = fieldNorm;
                    fieldReferenceReady = true;
                }

                float normErrorFrac = fabsf(fieldNorm - referenceFieldNorm_uT) /
                                      (referenceFieldNorm_uT > 0.001f ? referenceFieldNorm_uT : 1.0f);
                magDisturbed = normErrorFrac > MAG_DISTURB_THRESHOLD_FRAC;

                if (!ahrsInitialized)
                {
                    initializeAhrsPose(accelX,
                                       accelY,
                                       accelZ,
                                       mx,
                                       my,
                                       mz,
                                       fusedYawDeg,
                                       yawWasRestoredFromNv && yawInitialized);

                    if (!yawInitialized)
                    {
                        fusedYawDeg = quaternionYawDeg();
                        yawInitialized = true;
                        checkpointPersistentState(currentMillis, true);
                    }
                }

                float magWeight = 0.0f;
                if (!magDisturbed)
                {
                    referenceFieldNorm_uT = (0.98f * referenceFieldNorm_uT) + (0.02f * fieldNorm);
                    magWeight = MAHONY_MAG_WEIGHT;
                }

                updateAhrsMahony(dt,
                                 gyroXCorrected_dps * DEG2RAD,
                                 gyroYCorrected_dps * DEG2RAD,
                                 gyroZCorrected_dps * DEG2RAD,
                                 accelX,
                                 accelY,
                                 accelZ,
                                 mx,
                                 my,
                                 mz,
                                 magWeight);

                fusedYawDeg = quaternionYawDeg();
                heading_fused = fusedYawDeg;
            }

            if (currentMillis - lastPrintTime >= PRINT_INTERVAL)
            {
                lastPrintTime = currentMillis;

                float heading_output = yawInitialized ? fusedYawDeg : heading_fused;

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