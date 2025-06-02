#include <SoftwareSerial.h>

#define RANGEFINDER_PIN A0  // Connect the rangefinder to analog pin A0

// MSPv2 message structure
struct mspFrame {
    uint8_t size;
    uint8_t cmd;
};

struct mspRangeFinder {
    uint16_t distance;   // Distance in centimeters
};

SoftwareSerial serialPort(2, 3);  // RX pin (D2), TX pin (D3)

void setup() {
    Serial.begin(115200);
    serialPort.begin(115200);
}

void loop() {
    int range = analogRead(RANGEFINDER_PIN);  // Read distance from the rangefinder
    sendRangeFinderData(range);
    delay(100);  // Send data every 100 ms
}

void sendRangeFinderData(int range) {
    mspFrame header;
    mspRangeFinder rangeFinderData;
    uint8_t buf[9];

    // Prepare MSPv2 frame for sending rangefinder data
    header.size = sizeof(rangeFinderData) + 3;
    header.cmd = 145;  // Rangefinder MSP command (MSP_RANGEFINDER)
    buf[0] = '$';
    buf[1] = 'M';
    buf[2] = '<';
    memcpy(&buf[3], &header, sizeof(mspFrame));

    rangeFinderData.distance = range;  // Assign the distance value to the data structure
    memcpy(&buf[5], &rangeFinderData, sizeof(mspRangeFinder));

    uint16_t crc = calculateCRC(buf + 3, header.size - 3);
    buf[header.size + 2] = (crc >> 8) & 0xFF;
    buf[header.size + 3] = crc & 0xFF;

    serialPort.write(buf, header.size + 4);  // Send the MSPv2 frame over software serial port
}

uint16_t calculateCRC(uint8_t *data, uint8_t len) {
    uint16_t crc = 0;
    for (int i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}
