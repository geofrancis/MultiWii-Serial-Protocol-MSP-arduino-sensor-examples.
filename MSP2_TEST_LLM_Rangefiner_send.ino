#include <Wire.h>
#include <VL53L0X.h>  // For lidar sensor (replace with HC-SR04 if using ultrasonic)

#define SERIAL_PORT Serial  // Adjust for ESP32 (e.g., Serial1 or Serial2)
#define MSP_SET_RNGFND 150  // MSP V2 Rangefinder message ID

VL53L0X sensor;

void sendMSP(uint8_t cmd, uint8_t *payload, uint8_t payloadSize) {
    uint8_t checksum = cmd ^ payloadSize;
    
    SERIAL_PORT.write('$'); SERIAL_PORT.write('M'); SERIAL_PORT.write('<');
    SERIAL_PORT.write(payloadSize); SERIAL_PORT.write(cmd);
    
    for (uint8_t i = 0; i < payloadSize; i++) {
        SERIAL_PORT.write(payload[i]);
        checksum ^= payload[i];
    }
    
    SERIAL_PORT.write(checksum);
}

void sendRangefinderData() {
    uint16_t distance = sensor.readRangeSingleMillimeters();  // Read lidar distance in mm
    uint8_t payload[2];
    memcpy(&payload[0], &distance, 2);

    sendMSP(MSP_SET_RNGFND, payload, sizeof(payload));
}

void setup() {
    SERIAL_PORT.begin(115200);  // Match flight controller baud rate
    Wire.begin();
    sensor.init();
    sensor.setTimeout(500);
}

void loop() {
    sendRangefinderData();
    delay(1000);  // Send data every second
}