#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>                                                        
#include <WiFiClient.h>

const char* ssid = "TheWifiSSID";
const char* password = "TheWifiPassword";

WiFiClient client; 
WiFiServer wifiServer(9000);



struct particleSensorState_t {
    uint16_t avgPM25 = 0;
    boolean valid = false;
};

namespace SerialCom {
    constexpr static const uint8_t PIN_UART_RX = 4; // D2 on Wemos D1 Mini
    constexpr static const uint8_t PIN_UART_TX = 13; // UNUSED

    SoftwareSerial sensorSerial(PIN_UART_RX, PIN_UART_TX);

    uint8_t serialRxBuf[255];
    char outBuf[255];
    uint8_t rxBufIdx = 0;

    void setup() {
        sensorSerial.begin(9600);
    }

    void clearRxBuf() {
        memset(serialRxBuf, 0, sizeof(serialRxBuf));
        rxBufIdx = 0;
    }

    void parseState(particleSensorState_t& state) {
        sprintf(outBuf, "[%d,%d,%d,%d,%d,%d,%hhu,%hhu,%hhu,%hhu]\n",
        (serialRxBuf[3+0] << 8 | serialRxBuf[3+1]), //d1-2
        (serialRxBuf[3+2] << 8 | serialRxBuf[3+3]), //d3-4
        (serialRxBuf[3+4] << 8 | serialRxBuf[3+5]), //d5-6
        (serialRxBuf[3+6] << 8 | serialRxBuf[3+7]), //d7-8
        (serialRxBuf[3+8] << 8 | serialRxBuf[3+9]), //d9-10
        (serialRxBuf[3+10]<< 8 | serialRxBuf[3+11]), // d11-12
        serialRxBuf[3+12], // 13
        serialRxBuf[3+13], // 14
        serialRxBuf[3+14], // 15
        serialRxBuf[3+15] // 16
        );
        Serial.write(outBuf);

              if(client && client.connected()) {
                
                client.write(outBuf);
               
              } else {                                                                    
                if (client) {                                                             
                  client.stop();                                                          
                }                                                                                                                                       
                client = wifiServer.available();                                          
             }  


        clearRxBuf();
    }

    bool isValidHeader() {
        bool headerValid = serialRxBuf[0] == 0x16 && serialRxBuf[1] == 0x11 && serialRxBuf[2] == 0x0B;

        if (!headerValid) {
            Serial.println("Received message with invalid header.");
        }

        return headerValid;
    }

    bool isValidChecksum() {
        uint8_t checksum = 0;

        for (uint8_t i = 0; i < 20; i++) {
            checksum += serialRxBuf[i];
        }

        if (checksum != 0) {
            Serial.printf("Received message with invalid checksum. Expected: 0. Actual: %d\n", checksum);
        }

        return checksum == 0;
    }

    void handleUart(particleSensorState_t& state) {
        if (!sensorSerial.available()) {
            return;
        }

        Serial.print("Receiving:");
        while (sensorSerial.available()) {
            serialRxBuf[rxBufIdx++] = sensorSerial.read();
            Serial.print(".");

            // Without this delay, receiving data breaks for reasons that are beyond me
            delay(15);

            if (rxBufIdx >= 64) {
                clearRxBuf();
            }
        }
        Serial.println("Done.");

        if (isValidHeader() && isValidChecksum()) {
            parseState(state);
        } else {
            clearRxBuf();
        }
    }
}


void setup() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  wifiServer.begin();
  client = wifiServer.available();
  
  Serial.begin(9600);
  while(!Serial);

  SerialCom::setup();

}

particleSensorState_t state;
void loop() {
  SerialCom::handleUart(state);                                     
}
