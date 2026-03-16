#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#define BMM350_ADDR 0x14

/* I2C read (2 dummy bytes) */
bool readReg(uint8_t reg, uint8_t *data, uint8_t len)
{
    Wire.beginTransmission(BMM350_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0)
        return false;

    uint8_t total = len + 2;

    if (Wire.requestFrom(BMM350_ADDR, total) != total)
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

unsigned long lastReadTime = 0;
const unsigned long READ_INTERVAL = 100;  // 10 Hz for calibration

int sampleCount = 0;
const int TARGET_SAMPLES = 360;  // ~36 seconds at 10 Hz

float min_x = 999999.0f, max_x = -999999.0f;
float min_y = 999999.0f, max_y = -999999.0f;
float min_z = 999999.0f, max_z = -999999.0f;

void setup()
{
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("\n\n========================================");
    Serial.println("BMM350 MAGNETOMETER CALIBRATION");
    Serial.println("========================================");
    Serial.println("Initializing sensor...");
    
    Wire.begin();
    delay(100);
    
    // Soft reset
    writeReg(0x7E, 0xB6);
    delay(50);
    
    // Set normal mode
    writeReg(0x06, 0x01);
    writeReg(0x04, 0x04);
    delay(10);
    
    // Magnetic reset
    writeReg(0x06, 0x07);
    delay(20);
    
    Serial.println("Sensor initialized!");
    Serial.println("\n========================================");
    Serial.println("INSTRUCTIONS:");
    Serial.println("========================================");
    Serial.println("1. Keep the sensor LEVEL (horizontal)");
    Serial.println("2. When countdown reaches 0, start rotating");
    Serial.println("3. Rotate SLOWLY and SMOOTHLY 360 degrees");
    Serial.println("4. Try to maintain constant speed");
    Serial.println("5. Complete the full circle in ~30 seconds");
    Serial.println("========================================\n");
    
    for (int i = 5; i > 0; i--)
    {
        Serial.print("Starting in ");
        Serial.print(i);
        Serial.println(" seconds...");
        delay(1000);
    }
    
    Serial.println("\n*** START ROTATING NOW! ***\n");
    Serial.println("Sample,X_uT,Y_uT,Z_uT");
}

void loop()
{
    unsigned long currentMillis = millis();
    
    if (sampleCount >= TARGET_SAMPLES)
    {
        if (sampleCount == TARGET_SAMPLES)
        {
            Serial.println("\n========================================");
            Serial.println("CALIBRATION COMPLETE!");
            Serial.println("========================================");
            Serial.println("\nCalculated offsets (hard iron):");
            
            float offset_x = (max_x + min_x) / 2.0f;
            float offset_y = (max_y + min_y) / 2.0f;
            float offset_z = (max_z + min_z) / 2.0f;
            
            Serial.print("Hard Iron X: ");
            Serial.print(offset_x, 2);
            Serial.println(" uT");
            Serial.print("Hard Iron Y: ");
            Serial.print(offset_y, 2);
            Serial.println(" uT");
            Serial.print("Hard Iron Z: ");
            Serial.print(offset_z, 2);
            Serial.println(" uT");
            
            Serial.println("\nCalculated scale factors (soft iron):");
            
            float range_x = (max_x - min_x) / 2.0f;
            float range_y = (max_y - min_y) / 2.0f;
            float range_z = (max_z - min_z) / 2.0f;
            float avg_range = (range_x + range_y + range_z) / 3.0f;
            
            float scale_x = avg_range / range_x;
            float scale_y = avg_range / range_y;
            float scale_z = avg_range / range_z;
            
            Serial.print("Soft Iron X: ");
            Serial.println(scale_x, 3);
            Serial.print("Soft Iron Y: ");
            Serial.println(scale_y, 3);
            Serial.print("Soft Iron Z: ");
            Serial.println(scale_z, 3);
            
            Serial.println("\n========================================");
            Serial.println("Add these to your main.cpp:");
            Serial.println("========================================");
            Serial.print("const float HARD_IRON_X = ");
            Serial.print(offset_x, 2);
            Serial.println("f;");
            Serial.print("const float HARD_IRON_Y = ");
            Serial.print(offset_y, 2);
            Serial.println("f;");
            Serial.print("const float HARD_IRON_Z = ");
            Serial.print(offset_z, 2);
            Serial.println("f;");
            Serial.println();
            Serial.print("const float SOFT_IRON_X = ");
            Serial.print(scale_x, 3);
            Serial.println("f;");
            Serial.print("const float SOFT_IRON_Y = ");
            Serial.print(scale_y, 3);
            Serial.println("f;");
            Serial.print("const float SOFT_IRON_Z = ");
            Serial.print(scale_z, 3);
            Serial.println("f;");
            Serial.println("\nThen apply: x_corrected = (x - HARD_IRON_X) * SOFT_IRON_X");
            Serial.println("========================================\n");
            
            sampleCount++;  // Increment so we don't print again
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
            
            // Track min/max
            if (x_uT < min_x) min_x = x_uT;
            if (x_uT > max_x) max_x = x_uT;
            if (y_uT < min_y) min_y = y_uT;
            if (y_uT > max_y) max_y = y_uT;
            if (z_uT < min_z) min_z = z_uT;
            if (z_uT > max_z) max_z = z_uT;
            
            sampleCount++;
            
            // Print CSV format for analysis
            Serial.print(sampleCount);
            Serial.print(",");
            Serial.print(x_uT, 2);
            Serial.print(",");
            Serial.print(y_uT, 2);
            Serial.print(",");
            Serial.println(z_uT, 2);
            
            // Progress indicator
            if (sampleCount % 36 == 0)
            {
                int percent = (sampleCount * 100) / TARGET_SAMPLES;
                Serial.print("# Progress: ");
                Serial.print(percent);
                Serial.println("%");
            }
        }
    }
}