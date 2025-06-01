#include <Wire.h>

#define SERIAL_PORT Serial  // Use Serial or Serial1/Serial2 if using ESP32

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

void sendTelemetry() {
    int16_t roll = 100;    // Example roll value
    int16_t pitch = -50;   // Example pitch value
    int16_t yaw = 25;      // Example yaw value
    
    uint8_t imuPayload[6];
    memcpy(&imuPayload[0], &roll, 2);
    memcpy(&imuPayload[2], &pitch, 2);
    memcpy(&imuPayload[4], &yaw, 2);

    sendMSP(108, imuPayload, sizeof(imuPayload));  // MSP_ATTITUDE
}

void sendAltitude() {
    int32_t altitude = 150;  // Example altitude in cm
    
    uint8_t altPayload[4];
    memcpy(&altPayload[0], &altitude, 4);

    sendMSP(109, altPayload, sizeof(altPayload));  // MSP_ALTITUDE
}

void sendBatteryStatus() {
    uint8_t voltage = 12;  // Example battery voltage (12V)
    uint8_t rssi = 75;     // Example signal strength

    uint8_t batPayload[2] = {voltage, rssi};
    sendMSP(110, batPayload, sizeof(batPayload));  // MSP_ANALOG
}

void sendGPSData() {
    uint8_t fixStatus = 1;  // GPS fix acquired
    uint8_t numSatellites = 6;  // Example number of satellites
    int32_t lat = 55000000;  // Example latitude * 10^7
    int32_t lon = -30000000; // Example longitude * 10^7
    uint16_t speed = 30;  // Example speed in cm/s

    uint8_t gpsPayload[12];
    memcpy(&gpsPayload[0], &fixStatus, 1);
    memcpy(&gpsPayload[1], &numSatellites, 1);
    memcpy(&gpsPayload[2], &lat, 4);
    memcpy(&gpsPayload[6], &lon, 4);
    memcpy(&gpsPayload[10], &speed, 2);

    sendMSP(106, gpsPayload, sizeof(gpsPayload));  // MSP_GPS_RAW
}

void sendRCCommand() {
    uint16_t channels[8] = {1500, 1500, 1000, 1000, 2000, 1500, 1200, 1300}; // Example RC values

    uint8_t rcPayload[16];
    memcpy(&rcPayload[0], &channels, 16);

    sendMSP(200, rcPayload, sizeof(rcPayload));  // MSP_SET_RAW_RC
}

void sendDebugData() {
    int16_t debugValues[4] = {50, -30, 75, 0};  // Example debug values
    
    uint8_t debugPayload[8];
    memcpy(&debugPayload[0], &debugValues, 8);

    sendMSP(254, debugPayload, sizeof(debugPayload));  // MSP_DEBUG
}

void setup() {
    SERIAL_PORT.begin(115200);  // Adjust baud rate for flight controller
}

void loop() {
    sendTelemetry();
    sendAltitude();
    sendBatteryStatus();
    sendGPSData();
    sendRCCommand();
    sendDebugData();

    delay(1000);  // Send data every second
}