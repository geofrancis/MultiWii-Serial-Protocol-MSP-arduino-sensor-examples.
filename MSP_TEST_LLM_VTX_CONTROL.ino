#define SERIAL_PORT Serial  // Adjust for ESP32 (e.g., Serial1 or Serial2)

// MSP VTX Control Message IDs
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

void setVTXConfig(uint8_t band, uint8_t channel, uint8_t power, uint8_t pitMode) {
    uint8_t payload[4] = {band, channel, power, pitMode};
    sendMSP(MSP_SET_VTX_CONFIG, payload, sizeof(payload));
}

void setup() {
    SERIAL_PORT.begin(115200);  // Match flight controller baud rate
}

void loop() {
    setVTXConfig(2, 5, 3, 0);  // Example: Band 2, Channel 5, Power Level 3, Pit Mode Off
    delay(5000);  // Adjust settings every 5 seconds
}