
// Define predefined data for each MSP data type
uint8_t mspIdentData[4] = {1, 2, 3, 4};
uint16_t mspStatusData = 5;
int16_t mspRawIMUData[9] = {6, 7, 8, 9, 10, 11, 12, 13, 14};
uint16_t mspRCData[8] = {15, 16, 17, 18, 19, 20, 21, 22};
int32_t mspRawGPSData[8] = {23, 24, 25, 26, 27, 28, 29, 30};
int16_t mspCompGPSSpeed = 31;
uint16_t mspCompGPSHeading = 32;
int16_t mspAttitudeData[2] = {33, 34};
int32_t mspAltitudeData = 35;
uint16_t mspAnalogData[6] = {36, 37, 38, 39, 40, 41};
uint16_t mspMotorData[8] = {42, 43, 44, 45, 46, 47, 48, 49};

// Define MSP commands for each data type
const uint8_t MSP_IDENT = 100;
const uint8_t MSP_STATUS = 101;
const uint8_t MSP_RAW_IMU = 102;
const uint8_t MSP_RC = 103;
const uint8_t MSP_RAW_GPS = 106;
const uint8_t MSP_COMP_GPS = 107;
const uint8_t MSP_ATTITUDE = 108;
const uint8_t MSP_ALTITUDE = 109;
const uint8_t MSP_ANALOG = 110;
const uint8_t MSP_MOTOR = 112;


void setup() {
    // Initialize serial communication with flight controller
    Serial.begin(115200);
}

void loop() {
    // Send example data for each MSP data type
    sendMSP(MSP_IDENT, (uint8_t*)mspIdentData, sizeof(mspIdentData));
    delay(100);
    sendMSP(MSP_STATUS, (uint8_t*)&mspStatusData, sizeof(mspStatusData));
    delay(100);
    sendMSP(MSP_RAW_IMU, (uint8_t*)mspRawIMUData, sizeof(mspRawIMUData));
    delay(100);
    sendMSP(MSP_RC, (uint8_t*)mspRCData, sizeof(mspRCData));
    delay(100);
    sendMSP(MSP_RAW_GPS, (uint8_t*)mspRawGPSData, sizeof(mspRawGPSData));
    delay(100);
    sendMSP(MSP_COMP_GPS, (uint8_t*)&mspCompGPSSpeed, sizeof(mspCompGPSSpeed) + sizeof(mspCompGPSHeading));
    delay(100);
    sendMSP(MSP_ATTITUDE, (uint8_t*)mspAttitudeData, sizeof(mspAttitudeData));
    delay(100);
    sendMSP(MSP_ALTITUDE, (uint8_t*)&mspAltitudeData, sizeof(mspAltitudeData));
    delay(100);
    sendMSP(MSP_ANALOG, (uint8_t*)mspAnalogData, sizeof(mspAnalogData));
    delay(100);
    sendMSP(MSP_MOTOR, (uint8_t*)mspMotorData, sizeof(mspMotorData));
    delay(1000); // Send data every second
}

void sendMSP(uint8_t command, uint8_t* data, uint8_t length) {
    Serial.write('$');
    Serial.write('M');
    Serial.write('<');
    Serial.write((length & 0xFF00) >> 8);
    Serial.write(length & 0xFF);
    Serial.write(command);
    for (int i = 0; i < length; i++) {
        Serial.write(data[i]);
    }
    uint16_t crc = calculateCRC(command, data, length);
    Serial.write((crc >> 8) & 0xFF);
    Serial.write(crc & 0xFF);
}

uint16_t calculateCRC(uint8_t command, uint8_t* data, uint8_t length) {
    uint16_t crc = 0;
    crc ^= (command ^ data[0] ^ length);
    for (int i = 1; i < length; i++) {
        crc ^= data[i];
    }
    return crc;
}
