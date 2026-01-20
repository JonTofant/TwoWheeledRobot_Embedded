#include <Bluepad32.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// === Configuration ===
const char* apSSID = "2WheelRobotAP";
const char* apPassword = "robot123";
const char* udpAddress = "192.168.4.2";
const int udpPort = 12345;

#define UART_TX_PIN 17
#define UART_RX_PIN 16
#define SERIAL_BAUD 115200 

// === Protocol Definitions ===
#define TELEMETRY_SOF 0xAA
#define MAX_PAYLOAD_SIZE 250 
#define CTRL_TO_STM32_START_BYTE 0xAA

// === Global Objects ===
WiFiUDP udp;
HardwareSerial SerialPort(1); 
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// === Controller Callbacks (FIXED NAMES HERE) ===
void onConnectedController(ControllerPtr ctl) {
    // Corrected function name: getModelName()
    Serial.printf("Controller connected: %s\n", ctl->getModelName().c_str());
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            myControllers[i] = ctl;
            break;
        }
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    // Corrected function name: getModelName()
    Serial.printf("Controller disconnected: %s\n", ctl->getModelName().c_str());
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            myControllers[i] = nullptr;
            break;
        }
    }
}

void TaskController(void *pvParameters) {
    for(;;) {
        if (BP32.update()) {
            for (auto ctl : myControllers) {
                if (ctl && ctl->isConnected() && ctl->hasData() && ctl->isGamepad()) {
                    
                    int16_t axisX     = ctl->axisX();
                    uint16_t throttle  = ctl->throttle();
                    uint16_t brake     = ctl->brake();
                    uint8_t aBtn      = (uint8_t)(ctl->a() ? 1 : 0);
                    uint8_t dpad      = (uint8_t)ctl->dpad();
                    // Just take the first 8 bits of the misc buttons
                    uint8_t miscLow   = (uint8_t)(ctl->miscButtons() & 0xFF); 
                    
                    // Checksum calculation (Sum of bytes 1 through 9)
                    uint8_t checksum = 0;
                    checksum += (uint8_t)(axisX & 0xFF) + (uint8_t)((axisX >> 8) & 0xFF);
                    checksum += (uint8_t)(throttle & 0xFF) + (uint8_t)((throttle >> 8) & 0xFF);
                    checksum += (uint8_t)(brake & 0xFF) + (uint8_t)((brake >> 8) & 0xFF);
                    checksum += aBtn + dpad + miscLow;

                    // Send 11-byte packet
                    SerialPort.write(CTRL_TO_STM32_START_BYTE); // 0
                    SerialPort.write((uint8_t)(axisX & 0xFF));   // 1
                    SerialPort.write((uint8_t)(axisX >> 8));    // 2
                    SerialPort.write((uint8_t)(throttle & 0xFF));// 3
                    SerialPort.write((uint8_t)(throttle >> 8)); // 4
                    SerialPort.write((uint8_t)(brake & 0xFF));   // 5
                    SerialPort.write((uint8_t)(brake >> 8));    // 6
                    SerialPort.write(aBtn);                     // 7
                    SerialPort.write(dpad);                     // 8
                    SerialPort.write(miscLow);                  // 9
                    SerialPort.write(checksum);                 // 10
                }
            }
        }
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

// === Task: UART to UDP (Telemetry Bridge) ===
void TaskUARTtoUDP(void *pvParameters) {
    static uint8_t payloadBuffer[MAX_PAYLOAD_SIZE];
    static int rxIdx = 0;
    static uint8_t expectedLen = 0;
    
    enum State { WAITING_FOR_SOF, READING_LEN, READING_PAYLOAD, READING_CRC };
    static State state = WAITING_FOR_SOF;

    uint32_t packetsSent = 0;
    uint32_t lastReport = 0;

    for(;;) {
        while(SerialPort.available()) {
            uint8_t b = SerialPort.read();

            switch(state) {
                case WAITING_FOR_SOF:
                    if (b == TELEMETRY_SOF) state = READING_LEN;
                    break;

                case READING_LEN:
                    expectedLen = b;
                    if (expectedLen > 0 && expectedLen <= MAX_PAYLOAD_SIZE) {
                        rxIdx = 0;
                        state = READING_PAYLOAD;
                    } else {
                        state = WAITING_FOR_SOF;
                    }
                    break;

                case READING_PAYLOAD:
                    payloadBuffer[rxIdx++] = b;
                    if (rxIdx >= expectedLen) state = READING_CRC;
                    break;

                case READING_CRC:
                    uint8_t calcCrc = 0;
                    for(int i=0; i<expectedLen; i++) calcCrc += payloadBuffer[i];

                    if (b == calcCrc) {
                        udp.beginPacket(udpAddress, udpPort);
                        udp.write(TELEMETRY_SOF);
                        udp.write(expectedLen);
                        udp.write(payloadBuffer, expectedLen);
                        udp.write(b); 
                        udp.endPacket();
                        packetsSent++;
                    }
                    state = WAITING_FOR_SOF;
                    break;
            }
        }

        if (millis() - lastReport > 5000) {
            Serial.printf("[Bridge Status] UDP Packets Sent: %u\n", packetsSent);
            lastReport = millis();
        }
        vTaskDelay(1 / portTICK_PERIOD_MS); 
    }
}

void setup() {
    Serial.begin(115200);
    SerialPort.begin(SERIAL_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

    WiFi.softAP(apSSID, apPassword);
    udp.begin(udpPort);

    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.enableVirtualDevice(false);
    
    xTaskCreate(TaskController, "CtrlTask", 4096, NULL, 2, NULL);
    xTaskCreate(TaskUARTtoUDP, "BridgeTask", 4096, NULL, 1, NULL);
}

void loop() {
    vTaskDelete(NULL); 
}