#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <MS5837.h>
#include <utility/imumaths.h>
#include <ESP32Servo.h>

// WiFi Configuration
const char* ssid = "LoveMiw";
const char* password = "66444664";

// Webserver
WebServer server(80);

// กำหนดพิน
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

// เพิ่มตัวแปรควบคุมสถานะการทำงาน
bool isRunning = false;
float targetDepth = 0.0;  // Target depth in meters

// ค่า PID สำหรับแกน Y และ Z
float kP_Y = 2.0;
float kI_Y = 0.1;
float kD_Y = 0.5;
float kP_Z = 2.0;
float kI_Z = 0.1;
float kD_Z = 0.5;
float kP_D = 2.0;  // Depth P gain
float kI_D = 0.1;  // Depth I gain
float kD_D = 0.5;  // Depth D gain

#define TARGET_ANGLE_Y 0.0
#define TARGET_ANGLE_Z 0.0

// โครงสร้างข้อมูลสำหรับค่า PID
struct PIDOutput {
    float frontRight;
    float frontLeft;
    float backRight;
    float backLeft;
    float verticalThrust;  // Added for depth control
    
    PIDOutput() : frontRight(0), frontLeft(0), backRight(0), backLeft(0), verticalThrust(0) {}
    PIDOutput(float fr, float fl, float br, float bl, float vt)
        : frontRight(fr), frontLeft(fl), backRight(br), backLeft(bl), verticalThrust(vt) {}
};

// ตัวแปร PID
float errorY_prev = 0;
float errorZ_prev = 0;
float errorD_prev = 0;  // Previous depth error
float integralY = 0;
float integralZ = 0;
float integralD = 0;    // Depth integral
unsigned long lastTime = 0;

// อุปกรณ์
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);
MS5837 depthSensor;
Servo motor_FR, motor_FL, motor_BR, motor_BL;

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

// เพิ่มฟังก์ชันจัดการการ Start/Stop
void handleControl() {
    String action = server.arg("action");
    
    if (action == "start") {
        isRunning = true;
        server.send(200, "text/plain", "Started");
    } else if (action == "stop") {
        isRunning = false;
        // สั่งให้มอเตอร์ทุกตัวหยุดที่ค่า STOP_THROTTLE
        motor_FR.writeMicroseconds(STOP_THROTTLE);
        motor_FL.writeMicroseconds(STOP_THROTTLE);
        motor_BR.writeMicroseconds(STOP_THROTTLE);
        motor_BL.writeMicroseconds(STOP_THROTTLE);
        server.send(200, "text/plain", "Stopped");
    }
}

void handleUpdate() {
    if (server.hasArg("kp_y")) {
        kP_Y = server.arg("kp_y").toFloat();
        Serial.print("Updated kP_Y: ");
        Serial.println(kP_Y);
    }
    if (server.hasArg("ki_y")) {
        kI_Y = server.arg("ki_y").toFloat();
        Serial.print("Updated kI_Y: ");
        Serial.println(kI_Y);
    }
    if (server.hasArg("kd_y")) {
        kD_Y = server.arg("kd_y").toFloat();
        Serial.print("Updated kD_Y: ");
        Serial.println(kD_Y);
    }
    if (server.hasArg("kp_z")) {
        kP_Z = server.arg("kp_z").toFloat();
        Serial.print("Updated kP_Z: ");
        Serial.println(kP_Z);
    }
    if (server.hasArg("ki_z")) {
        kI_Z = server.arg("ki_z").toFloat();
        Serial.print("Updated kI_Z: ");
        Serial.println(kI_Z);
    }
    if (server.hasArg("kd_z")) {
        kD_Z = server.arg("kd_z").toFloat();
        Serial.print("Updated kD_Z: ");
        Serial.println(kD_Z);
    }
    if (server.hasArg("kp_D")) {
        kP_D = server.arg("kp_D").toFloat();
        Serial.print("Updated kP_D: ");
        Serial.println(kP_D);
    }
    if (server.hasArg("ki_D")) {
        kI_D = server.arg("ki_D").toFloat();
        Serial.print("Updated kI_D: ");
        Serial.println(kI_D);
    }
    if (server.hasArg("kd_D")) {
        kD_D = server.arg("kd_D").toFloat();
        Serial.print("Updated kD_D: ");
        Serial.println(kD_D);
    }
    if (server.hasArg("target_depth")) {
        targetDepth = server.arg("target_depth").toFloat();
        Serial.print("Updated targetDepth: ");
        Serial.println(targetDepth);
    }
    
    String html = "<!DOCTYPE html>"
                 "<html><head>"
                 "<title>AUV PID Tuner</title>"
                 "<style>"
                 ".control-buttons { margin: 20px 0; }"
                 ".control-buttons button { padding: 10px 20px; margin: 0 10px; cursor: pointer; }"
                 ".start-btn { background-color: #4CAF50; color: white; border: none; }"
                 ".stop-btn { background-color: #f44336; color: white; border: none; }"
                 "</style>"
                 "<script>"
                 "function controlQuad(action) {"
                 "  fetch('/control?action=' + action) "
                 "    .then(response => response.text())"
                 "    .then(data => alert('Command: ' + action + ' - ' + data));"
                 "} "
                 "</script>"
                 "</head>"
                 "<body>"
                 "<h1>AUV PID Parameter Tuner</h1>"
                 "<div class='control-buttons'>"
                 "<button class='start-btn' onclick='controlQuad(\"start\")'>Start</button>"
                 "<button class='stop-btn' onclick='controlQuad(\"stop\")'>Stop</button>"
                 "</div>"
                 "<form action='/update' method='GET'>"
                 "<h2>Y Axis (Pitch)</h2>"
                 "Kp_Y: <input type='text' name='kp_y' value='" + String(kP_Y) + "'><br>"
                 "Ki_Y: <input type='text' name='ki_y' value='" + String(kI_Y) + "'><br>"
                 "Kd_Y: <input type='text' name='kd_y' value='" + String(kD_Y) + "'><br>"
                 "<h2>Z Axis (Roll)</h2>"
                 "Kp_Z: <input type='text' name='kp_z' value='" + String(kP_Z) + "'><br>"
                 "Ki_Z: <input type='text' name='ki_z' value='" + String(kI_Z) + "'><br>"
                 "Kd_Z: <input type='text' name='kd_z' value='" + String(kD_Z) + "'><br>"
                 "<h2>Dept</h2>"
                 "Kp_D: <input type='text' name='kp_D' value='" + String(kP_D) + "'><br>"
                 "Ki_D: <input type='text' name='ki_D' value='" + String(kI_D) + "'><br>"
                 "Kd_D: <input type='text' name='kd_D' value='" + String(kD_D) + "'><br>"
                 "<h2>Target Depth</h2>"
                 "Target Depth: <input type='text' name='target_depth' value='" + String(targetDepth) + "'><br>"
                 "<input type='submit' value='Update PID'>"
                 "</form>"
                 "<hr>"
                 "<h2>Last Updated Values:</h2>"
                 "<div style='color: green;'>"
                 "<h3>Y Axis</h3>"
                 "Kp: " + String(kP_Y) + "<br>"
                 "Ki: " + String(kI_Y) + "<br>"
                 "Kd: " + String(kD_Y) + "<br>"
                 "<h3>Z Axis</h3>"
                 "Kp: " + String(kP_Z) + "<br>"
                 "Ki: " + String(kI_Z) + "<br>"
                 "Kd: " + String(kD_Z) + "<br>"
                 "<h3>Dept</h3>"
                 "Kp: " + String(kP_D) + "<br>"
                 "Ki: " + String(kI_D) + "<br>"
                 "Kd: " + String(kD_D) + "<br>"
                 "<h3>Target Depth</h3>"
                 "Target Depth: " + String(targetDepth) + "<br>"
                 "</div>"
                 "</body></html>";
    
    server.send(200, "text/html", html);
}

void handleRoot() {
    server.sendHeader("Location", "/update", true);
    server.send(302, "text/plain", "");
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
    // ตั้งค่า WiFi
    setupWiFi();

    // ตั้งค่า Web Server
    server.on("/", handleRoot);
    server.on("/update", handleUpdate);
    server.on("/control", handleControl);  // เพิ่ม endpoint สำหรับ start/stop
    server.begin();
    
    // ตั้งค่า I2C
    if(!bno.begin()) {
        Serial.println("BNO055 not detected!");
        while(1);
    }
    bno.setExtCrystalUse(true);

    if (!depthSensor.init()) {
        Serial.println("MS5837 not detected!");
        while(1);
    }
    depthSensor.setModel(MS5837::MS5837_30BA);
    depthSensor.init();
    depthSensor.setFluidDensity(997); // kg/m^3 for seawater
    
    delay(1000);
    Serial.println("Set up");
    
    // ตั้งค่า ESC
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
    
    // ตั้งค่าเริ่มต้นมอเตอร์
    motor_FR.writeMicroseconds(STOP_THROTTLE);
    motor_FL.writeMicroseconds(STOP_THROTTLE);
    motor_BR.writeMicroseconds(STOP_THROTTLE);
    motor_BL.writeMicroseconds(STOP_THROTTLE);
    delay(2000);
    Serial.println("Start!");
}

PIDOutput calculatePID(float angleY, float angleZ, float depth) {
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    if (dt == 0) return PIDOutput();
    
    // Calculate orientation errors
    float errorY = TARGET_ANGLE_Y - angleY;
    float errorZ = TARGET_ANGLE_Z - angleZ;
    float errorD = targetDepth - depth;
    
    // Calculate integrals
    integralY += errorY * dt;
    integralZ += errorZ * dt;
    integralD += errorD * dt;
    
    // Constrain integrals
    integralY = constrain(integralY, -100, 100);
    integralZ = constrain(integralZ, -100, 100);
    integralD = constrain(integralD, -100, 100);
    
    // Calculate derivatives
    float derivativeY = (errorY - errorY_prev) / dt;
    float derivativeZ = (errorZ - errorZ_prev) / dt;
    float derivativeD = (errorD - errorD_prev) / dt;
    
    // Calculate PID outputs
    float outputY = kP_Y * errorY + kI_Y * integralY + kD_Y * derivativeY;
    float outputZ = kP_Z * errorZ + kI_Z * integralZ + kD_Z * derivativeZ;
    float outputD = kP_D * errorD + kI_D * integralD + kD_D * derivativeD;
    
    // Update previous errors
    errorY_prev = errorY;
    errorZ_prev = errorZ;
    errorD_prev = errorD;
    lastTime = now;
    
    // Calculate motor outputs including depth control
    PIDOutput output(
        constrain(BASE_THROTTLE - outputY - outputZ + outputD, MIN_THROTTLE, MAX_THROTTLE),
        constrain(BASE_THROTTLE - outputY + outputZ + outputD, MIN_THROTTLE, MAX_THROTTLE),
        constrain(BASE_THROTTLE + outputY - outputZ + outputD, MIN_THROTTLE, MAX_THROTTLE),
        constrain(BASE_THROTTLE + outputY + outputZ + outputD, MIN_THROTTLE, MAX_THROTTLE),
        outputD
    );
    
    return output;
}

void loop() {
    // จัดการ web server
    server.handleClient();
    
    sensors_event_t event;
    bno.getEvent(&event);
    
    depthSensor.read();

    float depth = depthSensor.depth();
    float angleY = event.orientation.y;
    float angleZ = event.orientation.z;
    
    if (isRunning) {
        PIDOutput motorSpeeds = calculatePID(angleY, angleZ, depth);
        
        // ส่งค่าควบคุมไปยังมอเตอร์
        motor_FR.writeMicroseconds(motorSpeeds.frontRight);
        motor_FL.writeMicroseconds(motorSpeeds.frontLeft);
        motor_BR.writeMicroseconds(motorSpeeds.backRight);
        motor_BL.writeMicroseconds(motorSpeeds.backLeft);
        
    }
    
    delay(10);
}