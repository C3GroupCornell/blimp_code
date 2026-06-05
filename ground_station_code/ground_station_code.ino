// Open Serial Monitor, input "w" to go foward
//                     input "a" to go left
//                     input "s" to go back
//                     input "d" to go right
//                     input "q" to turn left
//                     input "e" to turn right
//                     input "t" to go up
//                     input "y" to go down

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#define CHANNEL 5
#define PRINTSCANRESULTS 0
#define DELETEBEFOREPAIR 0
#define DEBUG 0

typedef struct __attribute__((packed)) ControllerTXdataStruct{
    uint8_t uid;
    char commandType[8];
    double M1,M2,M3,M4,M5,M6;
    char Signal;
} CTX;

typedef struct __attribute__((packed)) ControllerRXdataStruct{
    uint8_t uid;
    char commandType[8];
    bool isWorking;
    int16_t altitude;
    int8_t altitudeAccuracy;
    float roll, pitch, yaw;
    float ax,ay,az;
    float gx,gy,gz;
} CRX;

CTX TXMsg;
CRX RXMsg;

String Serial0_RX_String = "";
boolean Serial0_RX_String_Complete = false;
uint32_t time_got_RX_String = 0;

esp_now_peer_info_t peripheralInfo;

double PD_pwm;
float pwm = 0.3;
double Hdata[2];
double target_height = 600.0;
double Kp_z = 0.00078;
double Kd_z = 0.005;

unsigned long start = 0;
unsigned long end;
bool isPaired = false;
bool isMoved = true;
bool isControlling = 0;
bool isRecording = 0;

bool ScanForPeripheral() {
    int16_t scanResults = WiFi.scanNetworks(false, false, false, 300, CHANNEL);
    bool peripheralFound = 0;
    memset(&peripheralInfo, 0, sizeof(peripheralInfo));

    if (scanResults == 0) {
        Serial.println("No WiFi devices in AP Mode found");
    } else {
        for (int i = 0; i < scanResults; ++i) {
            String SSID = WiFi.SSID(i);
            int32_t RSSI = WiFi.RSSI(i);
            String BSSIDstr = WiFi.BSSIDstr(i);
            delay(10);
            if (SSID == "Blimpy_McBlimpface") {
                Serial.println("Found Peripheral: " + SSID);
                int mac[6];
                if ( 6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x",  &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5] ) ) {
                    for (int ii = 0; ii < 6; ++ii ) {
                        peripheralInfo.peer_addr[ii] = (uint8_t) mac[ii];
                    }
                }
                peripheralInfo.channel = CHANNEL;
                peripheralInfo.encrypt = 0;
                peripheralFound = 1;
                break;
            }
        }
    }

    if (!peripheralFound) {
        Serial.println("Peripheral Not Found, trying again.");
        digitalWrite(1, LOW);
    }

    WiFi.scanDelete();
    return peripheralFound;
}

bool managePeripheral() {
    if (peripheralInfo.channel == CHANNEL) {
        if (DELETEBEFOREPAIR) {
          deletePeer();
        }
        bool exists = esp_now_is_peer_exist(peripheralInfo.peer_addr);
        if (exists) {
          return true;
        } else {
            esp_err_t addStatus = esp_now_add_peer(&peripheralInfo);
            if (addStatus == ESP_OK) {
                Serial.println("Pair success");
                return true;
            } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
                Serial.println("ESPNOW Not Init");
                return false;
            } else if (addStatus == ESP_ERR_ESPNOW_ARG) {
                Serial.println("Invalid Argument");
                return false;
            } else if (addStatus == ESP_ERR_ESPNOW_FULL) {
                Serial.println("Peer list full");
                return false;
            } else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) {
                Serial.println("Out of memory");
                return false;
            } else if (addStatus == ESP_ERR_ESPNOW_EXIST) {
                return true;
            } else {
                Serial.println("Not sure what happened");
                return false;
            }
        }
    } else {
        Serial.println("No Peripheral found to process");
        return false;
    }
}

void deletePeer() {
    esp_err_t delStatus = esp_now_del_peer(peripheralInfo.peer_addr);
    if (delStatus != ESP_OK) {
        Serial.println("Peripheral delete failed");
    }
}

void sendData() {
    const uint8_t *peer_addr = peripheralInfo.peer_addr;
    esp_err_t result = esp_now_send(peer_addr, (uint8_t *) &TXMsg, sizeof(TXMsg));
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        digitalWrite(1, HIGH);
    } else {
        digitalWrite(1, LOW);
    }
}

void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
    memcpy(&RXMsg, incomingData, sizeof(RXMsg));
    uint8_t start[1] = {0xAA};
    Serial.write(start, 1);
    Serial.write((uint8_t*)&RXMsg, sizeof(RXMsg));
}

void setup() {
    Serial.begin(921600);
    delay(10);

    WiFi.mode(WIFI_STA);
    esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);

    while(!ScanForPeripheral())
        delay(2000);

    if (peripheralInfo.channel == CHANNEL) {
        isPaired = managePeripheral();
    }
}

void parseToTXMsg(String input) {
    int commaIndex = 0;
    int lastCommaIndex = 0;
    
    commaIndex = input.indexOf(',');
    TXMsg.uid = input.substring(lastCommaIndex, commaIndex).toInt();
    
    lastCommaIndex = commaIndex + 1;
    commaIndex = input.indexOf(',', lastCommaIndex);
    String command = input.substring(lastCommaIndex, commaIndex);
    command.toCharArray(TXMsg.commandType, command.length() + 1);
    
    TXMsg.M1 = input.substring(commaIndex + 1, input.indexOf(',', commaIndex + 1)).toFloat();
    commaIndex = input.indexOf(',', commaIndex + 1);

    TXMsg.M2 = input.substring(commaIndex + 1, input.indexOf(',', commaIndex + 1)).toFloat();
    commaIndex = input.indexOf(',', commaIndex + 1);

    TXMsg.M3 = input.substring(commaIndex + 1, input.indexOf(',', commaIndex + 1)).toFloat();
    commaIndex = input.indexOf(',', commaIndex + 1);

    TXMsg.M4 = input.substring(commaIndex + 1, input.indexOf(',', commaIndex + 1)).toFloat();
    commaIndex = input.indexOf(',', commaIndex + 1);

    TXMsg.M5 = input.substring(commaIndex + 1, input.indexOf(',', commaIndex + 1)).toFloat();
    commaIndex = input.indexOf(',', commaIndex + 1);

    TXMsg.M6 = input.substring(commaIndex + 1, input.indexOf(',', commaIndex + 1)).toFloat();
    commaIndex = input.indexOf(',', commaIndex + 1);

    String signalStr = input.substring(commaIndex + 1, input.indexOf(',', commaIndex + 1));
    if (signalStr.length() > 0) {
        TXMsg.Signal = signalStr.charAt(0);
    }
}

void loop() {
    if (isControlling) {
        TXMsg.uid = 0;
        strcpy(TXMsg.commandType, "SETM");
        TXMsg.M3 = 0.5 * PD_pwm;
        TXMsg.M4 = 0.5 * PD_pwm;
        TXMsg.M1 = 0.0;
        TXMsg.M2 = 0.0;
        TXMsg.M5 = 0.0;
        TXMsg.M6 = 0.0;
        sendData();
    }

    if (Serial0_RX_String_Complete) {
        String command = Serial0_RX_String.substring(0, Serial0_RX_String.length() - 1);
        parseToTXMsg(command);
        isMoved = true;
        sendData();
        Serial0_RX_String_Complete = false;
        Serial0_RX_String = "";
    } else {
        TXMsg.uid = 0;
        strcpy(TXMsg.commandType, "SETM");
        TXMsg.M1 = 0.0;
        TXMsg.M2 = 0.0;
        TXMsg.M3 = 0.0;
        TXMsg.M4 = 0.0;
        TXMsg.M5 = 0.0;
        TXMsg.M6 = 0.0;
        TXMsg.Signal = '0';
        Serial0_RX_String_Complete = false;
    }
    delay(10);
}

void serialEventRun() {
    while (Serial.available()) {
        char inChar = (char)Serial.read();
        Serial0_RX_String += inChar;
        if (inChar == '\n') {
            Serial0_RX_String_Complete = true;
        }
    }
}