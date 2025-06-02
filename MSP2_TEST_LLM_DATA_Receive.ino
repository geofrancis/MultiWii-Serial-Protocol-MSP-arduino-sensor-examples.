#define SERIAL_PORT Serial  // Use Serial1/Serial2 if using ESP32

// MSP V2 Message IDs
#define MSP_ATTITUDE 108
#define MSP_ALTITUDE 109
#define MSP_ANALOG 110
#define MSP_GPS_RAW 106
#define MSP_RC 105
#define MSP_DEBUG 254
#define MSP_VTX_CONFIG 88

void parseMSP(uint8_t cmd, uint8_t *payload, uint8_t payloadSize) {
    switch (cmd) {
        case MSP_ATTITUDE: {
            int16_t roll, pitch, yaw;
            memcpy(&roll, &payload[0], 2);
            memcpy(&pitch, &payload[2], 2);
            memcpy(&yaw, &payload[4], 2);
            Serial.print("Attitude - Roll: "); Serial.print(roll);
            Serial.print(", Pitch: "); Serial.print(pitch);
            Serial.print(", Yaw: "); Serial.println(yaw);
            break;
        }
        case MSP_ALTITUDE: {
            int32_t altitude;
            memcpy(&altitude, &payload[0], 4);
            Serial.print("Altitude: "); Serial.println(altitude);
            break;
        }
        case MSP_ANALOG: {
            uint8_t voltage, rssi;
            voltage = payload[0];
            rssi = payload[1];
            Serial.print("Battery Voltage: "); Serial.print(voltage);
            Serial.print("V, RSSI: "); Serial.println(rssi);
            break;
        }
        case MSP_GPS_RAW: {
            uint8_t fixStatus, numSatellites;
            int32_t lat, lon;
            uint16_t speed;
            fixStatus = payload[0];
            numSatellites = payload[1];
            memcpy(&lat, &payload[2], 4);
            memcpy(&lon, &payload[6], 4);
            memcpy(&speed, &payload[10], 2);
            Serial.print("GPS - Fix: "); Serial.print(fixStatus);
            Serial.print(", Satellites: "); Serial.print(numSatellites);
            Serial.print(", Lat: "); Serial.print(lat);
            Serial.print(", Lon: "); Serial.print(lon);
            Serial.print(", Speed: "); Serial.println(speed);
            break;
        }
        case MSP_RC: {
            uint16_t channels[8];
            memcpy(&channels, &payload[0], 16);
            Serial.print("RC Channels: ");
            for (int i = 0; i < 8; i++) {
                Serial.print(channels[i]); Serial.print(" ");
            }
            Serial.println();
            break;
        }
        case MSP_DEBUG: {
            int16_t debugValues[4];
            memcpy(&debugValues, &payload[0], 8);
            Serial.print("Debug Data: ");
            for (int i = 0; i < 4; i++) {
                Serial.print(debugValues[i]); Serial.print(" ");
            }
            Serial.println();
            break;
        }
        case MSP_VTX_CONFIG: {
            uint8_t band, channel, power, pitMode;
            band = payload[0];
            channel = payload[1];
            power = payload[2];
            pitMode = payload[3];
            Serial.print("VTX Config - Band: "); Serial.print(band);
            Serial.print(", Channel: "); Serial.print(channel);
            Serial.print(", Power: "); Serial.print(power);
            Serial.print(", Pit Mode: "); Serial.println(pitMode);
            break;
        }
        default:
            Serial.print("Unknown MSP Command: "); Serial.println(cmd);
            break;
    }
}

void receiveMSP() {
    if (SERIAL_PORT.available() > 3) {
        if (SERIAL_PORT.read() == '$' && SERIAL_PORT.read() == 'M') {
            char direction = SERIAL_PORT.read();
            uint8_t payloadSize = SERIAL_PORT.read();
            uint8_t cmd = SERIAL_PORT.read();
            uint8_t payload[payloadSize];

            for (uint8_t i = 0; i < payloadSize; i++) {
                payload[i] = SERIAL_PORT.read();
            }

            uint8_t checksum = SERIAL_PORT.read();
            uint8_t calculatedChecksum = cmd ^ payloadSize;
            for (uint8_t i = 0; i < payloadSize; i++) {
                calculatedChecksum ^= payload[i];
            }

            if (checksum == calculatedChecksum) {
                parseMSP(cmd, payload, payloadSize);
            } else {
                Serial.println("Checksum mismatch!");
            }
        }
    }
}

void setup() {
    SERIAL_PORT.begin(115200);  // Match flight controller baud rate
    Serial.println("MSP Receiver Ready...");
}

void loop() {
    receiveMSP();
}