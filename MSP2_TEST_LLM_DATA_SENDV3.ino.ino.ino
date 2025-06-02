#include <Wire.h>

#define SERIAL_PORT Serial  // Use Serial1/Serial2 if using ESP32

// MSP V2 Message IDs
#define MSP_ATTITUDE 108
#define MSP_ALTITUDE 109
#define MSP_ANALOG 110
#define MSP_GPS_RAW 106
#define MSP_SET_RAW_RC 200
#define MSP_DEBUG 254
#define MSP_SET_VTX_CONFIG 88
#define MSP_GET_VTX_CONFIG 89

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

    sendMSP(MSP_ATTITUDE, imuPayload, sizeof(imuPayload));
}

void sendAltitude() {
    int32_t altitude = 150;  // Example altitude in cm
    
    uint8_t altPayload[4];
    memcpy(&altPayload[0], &altitude, 4);

    sendMSP(MSP_ALTITUDE, altPayload, sizeof(altPayload));
}

void sendBatteryStatus() {
    uint8_t voltage = 12;  // Example battery voltage (12V)
    uint8_t rssi = 75;     // Example signal strength

    uint8_t batPayload[2] = {voltage, rssi};
    sendMSP(MSP_ANALOG, batPayload, sizeof(batPayload));
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

    sendMSP(MSP_GPS_RAW, gpsPayload, sizeof(gpsPayload));
}

void sendRCCommand() {
    uint16_t channels[8] = {1500, 1500, 1000, 1000, 2000, 1500, 1200, 1300}; // Example RC values

    uint8_t rcPayload[16];
    memcpy(&rcPayload[0], &channels, 16);

    sendMSP(MSP_SET_RAW_RC, rcPayload, sizeof(rcPayload));
}

void sendDebugData() {
    int16_t debugValues[4] = {50, -30, 75, 0};  // Example debug values
    
    uint8_t debugPayload[8];
    memcpy(&debugPayload[0], &debugValues, 8);

    sendMSP(MSP_DEBUG, debugPayload, sizeof(debugPayload));
}

void setVTXConfig(uint8_t band, uint8_t channel, uint8_t power, uint8_t pitMode) {
    uint8_t payload[4] = {band, channel, power, pitMode};
    sendMSP(MSP_SET_VTX_CONFIG, payload, sizeof(payload));
}

void setup() {
    SERIAL_PORT.begin(115200);  // Match flight controller baud rate
}

void loop() {
    sendTelemetry();
    sendAltitude();
    sendBatteryStatus();
    sendGPSData();
    sendRCCommand();
    sendDebugData();
    setVTXConfig(2, 5, 3, 0);  // Example: Band 2, Channel 5, Power Level 3, Pit Mode Off

    delay(1000);  // Send data every second
}