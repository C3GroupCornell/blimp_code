#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "Wire.h"
#include "QMI8658.h"
#include "Adafruit_VL53L1X.h"

#define CHANNEL 5               // 通讯信道
#define CALIBRATE 0

// VCSEL 激光测高
#define IRQ_PIN 9               
#define XSHUT_PIN 10
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);
int16_t distance;               // 读取激光测搞的距离
int16_t current_distance;       // 最新一次读取的有效距离

//MIN MAX
#define MIN(i, j) (((i) < (j)) ? (i) : (j))
#define MAX(i, j) (((i) > (j)) ? (i) : (j))

//count
int count = 0;

// IMU
QMI8658 qmi8658;
float posX = 115;
float posY = 155;
float last_posX = posX;
float last_posY = posY;
// IMU new
float pitch;
float roll;
float yaw;
float ax_bias = 0.1387;
float ay_bias = -1.5096;
float az_bias = -0.7623;
float wx_offset = -0.4024;
float wy_offset = -0.1609;
float wz_offset = -0.0186;
float ax_scale = 1;
float ay_scale = 1;
float az_scale = 1;
float acc[3];
float gyro[3];  

unsigned long last_time = 0;


//电机PWM引脚定义 
uint16_t mtr1PWM = 2;
uint16_t mtr2PWM = 4;
uint16_t mtr3PWM = 5;
uint16_t mtr4PWM = 6;
uint16_t mtr5PWM = 7;
uint16_t mtr6PWM = 8;

// Global copy of controller
esp_now_peer_info_t controller;


//ESP-NOW 数据
boolean ESP_RX_Complete = false;    //ESP-NOW接收完成标志
bool isPaired = false;              //ESP-NOW是否已经配对

typedef struct __attribute__((packed)) PeripheralRXdataStruct{
    uint8_t uid;
    char commandType[8];
    double M1,M2,M3,M4,M5,M6;
    char Signal;
}PRX;
PRX RXMsg; //接收数据

typedef struct __attribute__((packed)) PeripheralTXdataStruct{
    uint8_t uid;
    char commandType[8];
    bool isWorking;
    int16_t altitude;
    int8_t altitudeAccuracy;
    float roll, pitch, yaw;
    float ax,ay,az;
    float gx,gy,gz;
}PTX;
PTX TXMsg; //发送数据


// Init ESP Now with fallback
void InitESPNow() {
    WiFi.disconnect();
    if (esp_now_init() == ESP_OK) {
        // Serial.println("ESPNow Init Success");
    }
    else {
        // Serial.println("ESPNow Init Failed");
        ESP.restart();
    }
}

// config AP SSID
void configDeviceAP() {
    const char *SSID = "Blimpy_McBlimpface";
    bool result = WiFi.softAP(SSID, "Blimpy_McBlimpface_Password", CHANNEL, 0);
    if (!result) {
        Serial.println("AP Config failed.");
    } else {
        Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
        Serial.print("AP CHANNEL "); Serial.println(WiFi.channel());
    }
}


// callback when data is received from Controller
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int data_len) {
    // Only copy the amount we received (protects against smaller packets)
    size_t copy_len = (size_t)min(data_len, (int)sizeof(RXMsg));
    // zero the struct first to avoid leftover values when partial data arrives
    memset(&RXMsg, 0, sizeof(RXMsg));
    memcpy(&RXMsg, incomingData, copy_len);

    // Serial.println("RXMsg motor 1: "+String(RXMsg.M1));
    // Serial.println("RXMsg motor 2: "+String(RXMsg.M2));
    // Serial.println("RXMsg motor 3: "+String(RXMsg.M3));
    // Serial.println("RXMsg motor 4: "+String(RXMsg.M4));
    // Serial.println("RXMsg motor 5: "+String(RXMsg.M5));
    // Serial.println("RXMsg motor 6: "+String(RXMsg.M6));

    // Minimal work here: set a flag and return quickly.
    // If not paired, try to pair (but avoid heavy ops repeatedly).
    if (!isPaired) {
        memcpy(controller.peer_addr, mac_addr, 6);
        controller.channel = CHANNEL;
        controller.encrypt = false;
        if (!esp_now_is_peer_exist(mac_addr)) {
            if (esp_now_add_peer(&controller) == ESP_OK) {
                isPaired = true;
            }
        } else {
            isPaired = true;
        }
    }
}


// 发送信息给Controller
void sendData() {
    const uint8_t *peer_addr = controller.peer_addr;
    esp_err_t result = esp_now_send(peer_addr,  (uint8_t *) &TXMsg, sizeof(TXMsg));
}

// callback function that will be executed when data is sent to Controller
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS){
        digitalWrite(1, HIGH);  // turn the LED on (HIGH is the voltage level 3.3V)
    } else {
        digitalWrite(1, LOW);  // turn the LED off (OFF is the voltage level 0V)
    }
}


void setup() {
    last_time = millis();
    // intialize the serial port
    Serial.begin(921600);
    memset(&RXMsg, 0, sizeof(RXMsg));
    memset(&TXMsg, 0, sizeof(TXMsg));
    TXMsg.isWorking = 0;

    // Ensure direction pins are defined before any motor activity
    pinMode(47, OUTPUT);
    pinMode(48, OUTPUT);
    digitalWrite(47, LOW); // set a known safe direction state
    digitalWrite(48, LOW);

    delay(10);

    // initialize the vertical motor pin as an output
    pinMode(47, OUTPUT);
    pinMode(48, OUTPUT);

    // PWM电机输出初始化
    ledcAttachPin(mtr1PWM, 1); 
    ledcAttachPin(mtr2PWM, 2); 
    ledcAttachPin(mtr3PWM, 3); 
    ledcAttachPin(mtr4PWM, 4); 
    ledcAttachPin(mtr5PWM, 5); 
    ledcAttachPin(mtr6PWM, 6); 
    ledcSetup(1, 10000, 10); 
    ledcSetup(2, 10000, 10);
    ledcSetup(3, 10000, 10);
    ledcSetup(4, 10000, 10); 
    ledcSetup(5, 10000, 10);
    ledcSetup(6, 10000, 10);

    //停止所有电机100ms
    mtrWriteDuty(1, 0);
    mtrWriteDuty(2, 0); 
    mtrWriteDuty(3, 0); 
    mtrWriteDuty(4, 0); 
    mtrWriteDuty(5, 0); 
    mtrWriteDuty(6, 0); 
    Serial.println("PWM Channels Init OK, 10bit @ 10KHz, Duty Cycle Output = 0, hold for 100 ms...");
    delay(100); 

    // initialize the ESP-NOW communication in AP_STA mode
    WiFi.mode(WIFI_AP_STA);

    // configure device AP mode
    configDeviceAP();

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);

    // IMU 初始化
    Wire.begin(15,16);
    qmi8658.setAccelUnit_mps2(1);
    qmi8658.setGyroUnit_rads(1);
    if( qmi8658.begin()== 0){
        Serial.println("qmi8658_init fail");
    }

    // VCSEL 激光测高
    Wire.begin(15,16);
    Serial.println("Booting Altitude Sensor");
    if (! vl53.begin(0x29, &Wire)) {
        Serial.print(F("Error on init of VL sensor: "));
        Serial.println(vl53.vl_status);
        while (1) delay(10);
    }
    Serial.println(F("VL53L1X sensor OK!"));
    Serial.print(F("Sensor ID: 0x"));
    Serial.println(vl53.sensorID(), HEX);

    if (! vl53.startRanging()) {
        Serial.print(F("Couldn't start ranging: "));
        Serial.println(vl53.vl_status);
        while (1) delay(10);
    }
    Serial.println(F("Ranging started"));

    // Valid timing budgets: 15, 20, 33, 50, 100, 200 and 500ms!
    vl53.setTimingBudget(50);
}


unsigned long start = 0;
unsigned long end;

//设定全局控制字
/*
 -+ 1  2 +-
    / \
    \ /
 ++ 5  6 +-
*/
float pwm_1 = 0.0;
float pwm_2 = 0.0;
float pwm_3 = 0.0;
float pwm_4 = 0.0;
float pwm_5 = 0.0;
float pwm_6 = 0.0;


bool Teleop = 1;

void calculateOrientation(float ax, float ay, float az, 
                         float gx, float gy, float gz,
                         float &roll, float &pitch, float &yaw) {
    static float yawIntegrated = 0;
    static float pitchIntegrated = 0;
    static float rollIntegrated = 0;
    static unsigned long lastTime = 0;
    
    unsigned long currentTime = millis();
    if (lastTime > 0) {
        float dt = (currentTime - lastTime) / 1000.0;
        yawIntegrated += gz * dt;
        pitchIntegrated += gy * dt;
        rollIntegrated += gx * dt;
    }
    lastTime = currentTime;
    
    yaw = yawIntegrated;
    pitch = pitchIntegrated;
    roll = rollIntegrated;
    
    while (yaw > 180) yaw -= 2*M_PI;
    while (yaw < -180) yaw += 2*M_PI;
    while (pitch > 180) yaw -= 2*M_PI;
    while (pitch < -180) yaw += 2*M_PI;
    while (roll > 180) yaw -= 2*M_PI;
    while (roll < -180) yaw += 2*M_PI;
}

void calibrateIMU(QMI8658_Data &sensorData)
{
        float sums[6];
        int samples = 5000;
        for (int i=0;i<samples;i++)
        {
            float roll_cal,pitch_cal,yaw_cal;
            qmi8658.readSensorData(sensorData);

            float ax_out = sensorData.accelX;
            float ay_out = sensorData.accelY;
            float az_out = sensorData.accelZ;
            float gyrox_out = sensorData.gyroX;
            float gyroy_out = sensorData.gyroY;
            float gyroz_out = sensorData.gyroZ;

            sums[0] += gyrox_out;
            sums[1] += gyroy_out;
            sums[2] += gyroz_out;
            sums[3] += ax_out;
            sums[4] += ay_out;
            sums[5] += az_out;
            if(i>100 && i%100==0)
            {
                Serial.print(F("Calibration at n (/5000)"));
                Serial.println(i);
            }
            delay(1);
        }
        wx_offset  =     -sums[0]/samples;
        wy_offset  =     -sums[1]/samples;
        wz_offset  =     -sums[2]/samples;
        ax_bias    =     -sums[3]/samples;
        ay_bias    =     -sums[4]/samples;
        az_bias    = 9.81-sums[5]/samples;
}

void loop() {
    QMI8658_Data sensorData;

    if (CALIBRATE == 1 && count == 0) {
        Serial.println("Warming up IMU for 10 seconds");
        delay(10000);
        calibrateIMU(sensorData);
        count += 1;
    }

    qmi8658.readSensorData(sensorData);

    float ax_out = (sensorData.accelX + ax_bias) * ax_scale;
    float ay_out = (sensorData.accelY + ay_bias) * ay_scale;
    float az_out = (sensorData.accelZ + az_bias) * az_scale;
    TXMsg.ax = ax_out;
    TXMsg.ay = ay_out;
    TXMsg.az = az_out;

    float gyrox_out = sensorData.gyroX + wx_offset;
    float gyroy_out = sensorData.gyroY + wy_offset;
    float gyroz_out = sensorData.gyroZ + wz_offset;

    calculateOrientation(ax_out, ay_out, az_out, gyrox_out, gyroy_out, gyroz_out, roll, pitch, yaw);

    if (vl53.dataReady()) {
        distance = vl53.distance();
        if (distance == -1) {
            TXMsg.altitude = current_distance;
            TXMsg.altitudeAccuracy = -1;
        } else {
            TXMsg.altitude = distance;
            TXMsg.altitudeAccuracy = 1;
            current_distance = distance;
        }
        vl53.clearInterrupt();
    } else {
        TXMsg.altitude = current_distance;
        TXMsg.altitudeAccuracy = 0;
    }

    if (isPaired) {
        TXMsg.roll = roll;
        TXMsg.pitch = pitch;
        TXMsg.yaw = yaw;
        sendData();
    }

    // --------------------------
    // Command decoding from RXMsg.Signal
    // --------------------------
    if (RXMsg.Signal == 'r') {
      Teleop = 1;
    }
    if (RXMsg.Signal == 'w') {
      Teleop = 1;
      RXMsg.M1 = 0.3;
      RXMsg.M2 = 0.0;
      RXMsg.M5 = 0.3;
      RXMsg.M6 = 0.0;
    }
    if (RXMsg.Signal == 'a') {
      Teleop = 1;
      RXMsg.M1 = 0.3;
      RXMsg.M2 = 0.0;
      RXMsg.M5 = 0.0;
      RXMsg.M6 = 0.3;
    }
    if (RXMsg.Signal == 's') {
      Teleop = 1;
      RXMsg.M1 = 0.0;
      RXMsg.M2 = 0.3;
      RXMsg.M5 = 0.0;
      RXMsg.M6 = 0.3;
    }
    if (RXMsg.Signal == 'd') {
      Teleop = 1;
      RXMsg.M1 = 0.0;
      RXMsg.M2 = 0.3;
      RXMsg.M5 = 0.3;
      RXMsg.M6 = 0.0;
    }
    if (RXMsg.Signal == 'f') {
      Teleop = 1;
      RXMsg.M1 = 0.3;
      RXMsg.M2 = 0.0;
      RXMsg.M5 = 0.0;
      RXMsg.M6 = 0.0;
    }
    if (RXMsg.Signal == 'g') {
      Teleop = 1;
      RXMsg.M1 = 0.0;
      RXMsg.M2 = 0.3;
      RXMsg.M5 = 0.0;
      RXMsg.M6 = 0.0;
    }
    if (RXMsg.Signal == 't') {
      Teleop = 0;
      RXMsg.M3 = 0.1;
      RXMsg.M4 = 0.1;
    }
    if (RXMsg.Signal == 'y') {
      Teleop = 0;
      RXMsg.M3 = -0.2;
      RXMsg.M4 = -0.2;
    }

    // --------------------------
    // Horizontal motor writes
    // --------------------------
    if (Teleop == 1) {
      mtrWriteDuty(1, RXMsg.M1);
      mtrWriteDuty(2, RXMsg.M2);
      mtrWriteDuty(5, RXMsg.M5);
      mtrWriteDuty(6, RXMsg.M6);
    } else {
      mtrWriteDuty(1, 0);
      mtrWriteDuty(2, 0);
      mtrWriteDuty(5, 0);
      mtrWriteDuty(6, 0);
    }

    // --------------------------
    // Vertical motor control (M3, M4)
    // --------------------------
    // Motor 3
    if (RXMsg.M3 > 0) {
        digitalWrite(47, HIGH);
        mtrWriteDuty(3, RXMsg.M3);
    } else if (RXMsg.M3 < 0) {
        digitalWrite(47, LOW);
        mtrWriteDuty(3, 1+RXMsg.M3);
    } else {
        digitalWrite(47, LOW);
        mtrWriteDuty(3, 0);
    }

    // Motor 4
    if (RXMsg.M4 > 0) {
        digitalWrite(48, HIGH);
        mtrWriteDuty(4, RXMsg.M4);
    } else if (RXMsg.M4 < 0) {
        digitalWrite(48, LOW);
        mtrWriteDuty(4, 1+RXMsg.M4);
    } else {
        digitalWrite(48, LOW);
        mtrWriteDuty(4, 0);
    }

    // --------------------------
    // TXMsg.isWorking state machine based on Signal
    // --------------------------
    if (RXMsg.Signal == '0') {
      Teleop = 0;
      RXMsg.M3 = 0;
      RXMsg.M4 = 0;
      TXMsg.isWorking = 0;
    } else if (RXMsg.Signal == '\0') {
      TXMsg.isWorking = 0;
    } else {
      TXMsg.isWorking = 1;
    }
}


// ---------------函数 mtrWriteDuty()----------------------
// ---------------电机PWM占空比脉宽输出------------------
void mtrWriteDuty(int mtr_idx, double dutyCycle) {
    if (!isfinite(dutyCycle)) dutyCycle = 0.0;
    if (dutyCycle > 1.0) dutyCycle = 1.0;
    if (mtr_idx < 0 || mtr_idx > 15) return;
    int dutyInt = (int)round(dutyCycle * 1023.0);
    if (dutyInt < 0) dutyInt = 0;
    if (dutyInt > 1023) dutyInt = 1023;
    ledcWrite(mtr_idx, dutyInt);
}

// ---------------函数 mtrWritePWMus()----------------------
// ---------------电机PWM微秒ON-TIME脉宽输出------------------
void mtrWritePWMus(int mtr_idx, float on_time_us) {
    float dutyCycle = on_time_us/20000;
    ledcWrite(mtr_idx, dutyCycle*65535);
}

// ---------------函数 mtrWritePWMusInv()----------------------
// ---------------电机PWM微秒ON-TIME脉宽输出[反相]----------------
void mtrWritePWMusInv(int mtr_idx, float on_time_us) {
    float dutyCycle = 1- on_time_us/20000;
    ledcWrite(mtr_idx, dutyCycle*65535);
}