#include <WiFi.h>
#include <WebSocketsServer.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <ESP32Servo.h>
#include <ArduinoJson.h>

// WiFi Configuration
const char* ssid = "LoveMiw";
const char* password = "66444664";

// WebSocket
WebSocketsServer webSocket = WebSocketsServer(81);

// Pin definitions
#define I2C_SDA 21
#define I2C_SCL 22
#define MOTOR_FRONT_RIGHT 15
#define MOTOR_FRONT_LEFT 13
#define MOTOR_BACK_RIGHT 14
#define MOTOR_BACK_LEFT 26

#define MIN_THROTTLE 1000
#define MAX_THROTTLE 2000
#define BASE_THROTTLE 1510
#define STOP_THROTTLE 1505

bool isRunning = false;

// PID parameters
float kP_Y = 2.0;
float kI_Y = 0.1;
float kD_Y = 0.5;
float kP_Z = 2.0;
float kI_Z = 0.1;
float kD_Z = 0.5;

#define TARGET_ANGLE_Y 0.0
#define TARGET_ANGLE_Z 0.0

struct PIDOutput {
    float frontRight;
    float frontLeft;
    float backRight;
    float backLeft;
    
    PIDOutput() : frontRight(0), frontLeft(0), backRight(0), backLeft(0) {}
    PIDOutput(float fr, float fl, float br, float bl)
        : frontRight(fr), frontLeft(fl), backRight(br), backLeft(bl) {}
};

float errorY_prev = 0;
float errorZ_prev = 0;
float integralY = 0;
float integralZ = 0;
unsigned long lastTime = 0;
unsigned long lastWebSocketUpdate = 0;
const unsigned long WEBSOCKET_INTERVAL = 50;  // Increased update rate to 20Hz

TwoWire I2CBNO = TwoWire(0);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &I2CBNO);
Servo motor_FR, motor_FL, motor_BR, motor_BL;

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            Serial.printf("[%u] Disconnected!\n", num);
            break;
        case WStype_CONNECTED:
            Serial.printf("[%u] Connected!\n", num);
            break;
        case WStype_TEXT: {
            String text = String((char*)payload);
            StaticJsonDocument<200> doc;
            DeserializationError error = deserializeJson(doc, text);
            
            if (error) {
                Serial.println("Failed to parse JSON");
                return;
            }
            
            // Handle commands
            if (doc.containsKey("command")) {
                String command = doc["command"];
                if (command == "start") isRunning = true;
                else if (command == "stop") {
                    isRunning = false;
                    motor_FR.writeMicroseconds(STOP_THROTTLE);
                    motor_FL.writeMicroseconds(STOP_THROTTLE);
                    motor_BR.writeMicroseconds(STOP_THROTTLE);
                    motor_BL.writeMicroseconds(STOP_THROTTLE);
                }
            }
            
            // Handle PID updates
            if (doc.containsKey("pid")) {
                kP_Y = doc["pid"]["kp_y"] | kP_Y;
                kI_Y = doc["pid"]["ki_y"] | kI_Y;
                kD_Y = doc["pid"]["kd_y"] | kD_Y;
                kP_Z = doc["pid"]["kp_z"] | kP_Z;
                kI_Z = doc["pid"]["ki_z"] | kI_Z;
                kD_Z = doc["pid"]["kd_z"] | kD_Z;
            }
            break;
        }
    }
}

void setupWiFi() {
    WiFi.begin(ssid, password);
    Serial.println("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi Connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
}

void sendSensorData(float angleY, float angleZ) {
    if (millis() - lastWebSocketUpdate >= WEBSOCKET_INTERVAL) {
        StaticJsonDocument<200> doc;
        
        doc["angleY"] = angleY;
        doc["angleZ"] = angleZ;
        doc["pid"]["kp_y"] = kP_Y;
        doc["pid"]["ki_y"] = kI_Y;
        doc["pid"]["kd_y"] = kD_Y;
        doc["pid"]["kp_z"] = kP_Z;
        doc["pid"]["ki_z"] = kI_Z;
        doc["pid"]["kd_z"] = kD_Z;
        doc["isRunning"] = isRunning;
        
        String jsonString;
        serializeJson(doc, jsonString);
        webSocket.broadcastTXT(jsonString);
        
        lastWebSocketUpdate = millis();
    }
}

PIDOutput calculatePID(float angleY, float angleZ) {
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    if (dt == 0) return PIDOutput();
    
    float errorY = TARGET_ANGLE_Y - angleY;
    float errorZ = TARGET_ANGLE_Z - angleZ;
    
    integralY += errorY * dt;
    integralZ += errorZ * dt;
    
    integralY = constrain(integralY, -100, 100);
    integralZ = constrain(integralZ, -100, 100);
    
    float derivativeY = (errorY - errorY_prev) / dt;
    float derivativeZ = (errorZ - errorZ_prev) / dt;
    
    float outputY = kP_Y * errorY + kI_Y * integralY + kD_Y * derivativeY;
    float outputZ = kP_Z * errorZ + kI_Z * integralZ + kD_Z * derivativeZ;
    
    errorY_prev = errorY;
    errorZ_prev = errorZ;
    lastTime = now;
    
    PIDOutput output(
        constrain(BASE_THROTTLE - outputY - outputZ, MIN_THROTTLE, MAX_THROTTLE),
        constrain(BASE_THROTTLE - outputY + outputZ, MIN_THROTTLE, MAX_THROTTLE),
        constrain(BASE_THROTTLE + outputY - outputZ, MIN_THROTTLE, MAX_THROTTLE),
        constrain(BASE_THROTTLE + outputY + outputZ, MIN_THROTTLE, MAX_THROTTLE)
    );
    
    return output;
}

void setup() {
    Serial.begin(115200);
    
    setupWiFi();
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    
    I2CBNO.begin(I2C_SDA, I2C_SCL);
    if(!bno.begin()) {
        Serial.println("BNO055 not detected!");
        while(1);
    }
    delay(1000);
    bno.setExtCrystalUse(true);
    
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    
    motor_FR.setPeriodHertz(50);
    motor_FL.setPeriodHertz(50);
    motor_BR.setPeriodHertz(50);
    motor_BL.setPeriodHertz(50);
    
    motor_FR.attach(MOTOR_FRONT_RIGHT, MIN_THROTTLE, MAX_THROTTLE);
    motor_FL.attach(MOTOR_FRONT_LEFT, MIN_THROTTLE, MAX_THROTTLE);
    motor_BR.attach(MOTOR_BACK_RIGHT, MIN_THROTTLE, MAX_THROTTLE);
    motor_BL.attach(MOTOR_BACK_LEFT, MIN_THROTTLE, MAX_THROTTLE);
    
    motor_FR.writeMicroseconds(STOP_THROTTLE);
    motor_FL.writeMicroseconds(STOP_THROTTLE);
    motor_BR.writeMicroseconds(STOP_THROTTLE);
    motor_BL.writeMicroseconds(STOP_THROTTLE);
    delay(2000);
}

void loop() {
    webSocket.loop();
    
    sensors_event_t event;
    bno.getEvent(&event);
    
    float angleY = event.orientation.y;
    float angleZ = event.orientation.z;
    
    sendSensorData(angleY, angleZ);
    
    if (isRunning) {
        PIDOutput motorSpeeds = calculatePID(angleY, angleZ);
        
        motor_FR.writeMicroseconds(motorSpeeds.frontRight);
        motor_FL.writeMicroseconds(motorSpeeds.frontLeft);
        motor_BR.writeMicroseconds(motorSpeeds.backRight);
        motor_BL.writeMicroseconds(motorSpeeds.backLeft);
    }
    
    delay(10);
}