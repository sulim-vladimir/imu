#include "I2Cdev.h"
#include "MPU6050.h"
#include "TimerOne.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// I2C addresses:
// 0x68 - MPU6050 

MPU6050 mpu6050;

int16_t ax, ay, az;
int16_t gx, gy, gz;

#define LED_PIN 13
#define TRIGGER_PIN 3 // camera trigger pin

bool blinkState = false;
bool timerInterrupt = false;

const int imuFrequency = 200;
const int triggerFrequency = 20;
int triggerDivisor;
int triggerCount = 0;

uint8_t startByte;
const uint8_t packetSize = 17; // 2 bytes * 1 imu * (3 gyro axes + 3 accek axes) + 4 service bytes + 1 packet counter
uint8_t imuPacket[packetSize];

void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);

    // Setting of mpu6050 full range is done during initialization
    mpu6050.initialize();
    mpu6050.setRate(4); // Divisor corresponds to 200 Hz

    // Optimum offsets
    // X acceleration, Y acceleration, Z acceleration,
    // [-3027,-3026] --> [-9,9] 
    // [919,920] --> [-6,9]  
    // [1748,1748] --> [16384,16388] 
    // X gyro, Y gyro, and Z gyro, respectively
    // [1,2] --> [-1,4]  
    // [7,8] --> [-1,2]  
    // [67,68] --> [-1,1]

    mpu6050.setXAccelOffset(-3027);
    mpu6050.setYAccelOffset(919);
    mpu6050.setZAccelOffset(1748);
    mpu6050.setXGyroOffset(1);
    mpu6050.setYGyroOffset(7);
    mpu6050.setZGyroOffset(67);

    imuPacket[0] = '$';
    imuPacket[1] = 0x03;
    for (uint8_t i = 2; i < packetSize - 2; i++) {
      imuPacket[i] = 0;
    }
    imuPacket[packetSize - 2] = '\r';
    imuPacket[packetSize - 1] = '\n';

    Timer1.initialize(1000000 / imuFrequency);
    Timer1.attachInterrupt(callback);

    triggerDivisor = imuFrequency / triggerFrequency;

    pinMode(LED_PIN, OUTPUT);
    
    pinMode(TRIGGER_PIN, OUTPUT);
    digitalWrite(TRIGGER_PIN, LOW);
}

void callback() {
    timerInterrupt = true;
}

void fillMeasurements() {
    imuPacket[startByte + 0] = ax >> 8;
    imuPacket[startByte + 1] = ax & 0xFF;
    imuPacket[startByte + 2] = ay >> 8;
    imuPacket[startByte + 3] = ay & 0xFF;
    imuPacket[startByte + 4] = az >> 8;
    imuPacket[startByte + 5] = az & 0xFF;
    
    imuPacket[startByte + 6] = gx >> 8;
    imuPacket[startByte + 7] = gx & 0xFF;
    imuPacket[startByte + 8] = gy >> 8;
    imuPacket[startByte + 9] = gy & 0xFF;
    imuPacket[startByte + 10] = gz >> 8;
    imuPacket[startByte + 11] = gz & 0xFF;   
}

void loop() {
    if (!timerInterrupt) {
        return;
    }
    timerInterrupt = false;

    if (triggerCount == imuFrequency) {
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
          
        triggerCount = 0;          
    }

    if (triggerCount % triggerDivisor == 0) {
        digitalWrite(TRIGGER_PIN, HIGH);
    } else if ((triggerCount + 2) % triggerDivisor == 0) {
        digitalWrite(TRIGGER_PIN, LOW);
    }
    triggerCount++;

    startByte = 2;
    mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    fillMeasurements();

    Serial.write(imuPacket, packetSize); 

    imuPacket[packetSize - 3]++; // packetCount loops at 0xFF on purpose
}
