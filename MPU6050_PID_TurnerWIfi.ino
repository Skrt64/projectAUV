#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>

// WiFi Configuration
const char* ssid = "LoveMiw";
const char* password = "66444664";

// Webserver
WebServer server(80);

// กำหนดพิน
#define I2C_SDA 21
#define I2C_SCL 22
#define MOTOR_FRONT_RIGHT 25
#define MOTOR_FRONT_LEFT 26
#define MOTOR_BACK_RIGHT 27
#define MOTOR_BACK_LEFT 32

#define MIN_THROTTLE 1000
#define MAX_THROTTLE 2000
#define BASE_THROTTLE 1500

// ค่า PID สำหรับแกน Y และ Z
float kP_Y = 2.0, kI_Y = 0.1, kD_Y = 0.5;
float kP_Z = 2.0, kI_Z = 0.1, kD_Z = 0.5;

#define TARGET_ANGLE_Y 0.0
#define TARGET_ANGLE_Z 0.0

// โครงสร้างข้อมูลสำหรับค่า PID
struct PIDOutput {
    float frontRight, frontLeft, backRight, backLeft;
    PIDOutput() : frontRight(0), frontLeft(0), backRight(0), backLeft(0) {}
    PIDOutput(float fr, float fl, float br, float bl)
        : frontRight(fr), frontLeft(fl), backRight(br), backLeft(bl) {}
};

// ตัวแปร PID
float errorY_prev = 0, errorZ_prev = 0;
float integralY = 0, integralZ = 0;
unsigned long lastTime = 0;

// อุปกรณ์
Adafruit_MPU6050 mpu;
Servo motor_FR, motor_FL, motor_BR, motor_BL;

void setupWiFi() {
    WiFi.begin(ssid, password);
    Serial.println("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi Connected! IP: " + WiFi.localIP().toString());
}

void handleUpdate() {
    if (server.hasArg("kp_y")) kP_Y = server.arg("kp_y").toFloat();
    if (server.hasArg("ki_y")) kI_Y = server.arg("ki_y").toFloat();
    if (server.hasArg("kd_y")) kD_Y = server.arg("kd_y").toFloat();
    if (server.hasArg("kp_z")) kP_Z = server.arg("kp_z").toFloat();
    if (server.hasArg("ki_z")) kI_Z = server.arg("ki_z").toFloat();
    if (server.hasArg("kd_z")) kD_Z = server.arg("kd_z").toFloat();
    
    String html = "<!DOCTYPE html>"
                 "<html><head><title>Quadcopter PID Tuner</title></head>"
                 "<body>"
                 "<h1>Quadcopter PID Parameter Tuner</h1>"
                 "<form action='/update' method='GET'>"
                 "<h2>Y Axis (Pitch)</h2>"
                 "Kp_Y: <input type='text' name='kp_y' value='" + String(kP_Y) + "'><br>"
                 "Ki_Y: <input type='text' name='ki_y' value='" + String(kI_Y) + "'><br>"
                 "Kd_Y: <input type='text' name='kd_y' value='" + String(kD_Y) + "'><br>"
                 "<h2>Z Axis (Roll)</h2>"
                 "Kp_Z: <input type='text' name='kp_z' value='" + String(kP_Z) + "'><br>"
                 "Ki_Z: <input type='text' name='ki_z' value='" + String(kI_Z) + "'><br>"
                 "Kd_Z: <input type='text' name='kd_z' value='" + String(kD_Z) + "'><br>"
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
                 "</div>"
                 "</body></html>";
                 
    server.send(200, "text/html", html);
}

// Since we're showing everything in handleUpdate now, we can make handleRoot redirect there
void handleRoot() {
    server.sendHeader("Location", "/update", true);
    server.send(302, "text/plain", "");
}

void setup() {
    Serial.begin(115200);
    setupWiFi();
    server.on("/", handleRoot);
    server.on("/update", handleUpdate);
    server.begin();

    Wire.begin(I2C_SDA, I2C_SCL);
    if (!mpu.begin()) {
        Serial.println("MPU6050 not detected!");
        while (1);
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    delay(1000);

    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    motor_FR.attach(MOTOR_FRONT_RIGHT, MIN_THROTTLE, MAX_THROTTLE);
    motor_FL.attach(MOTOR_FRONT_LEFT, MIN_THROTTLE, MAX_THROTTLE);
    motor_BR.attach(MOTOR_BACK_RIGHT, MIN_THROTTLE, MAX_THROTTLE);
    motor_BL.attach(MOTOR_BACK_LEFT, MIN_THROTTLE, MAX_THROTTLE);
    motor_FR.writeMicroseconds(MIN_THROTTLE);
    motor_FL.writeMicroseconds(MIN_THROTTLE);
    motor_BR.writeMicroseconds(MIN_THROTTLE);
    motor_BL.writeMicroseconds(MIN_THROTTLE);
    delay(2000);
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
    return PIDOutput(
        constrain(BASE_THROTTLE - outputY - outputZ, MIN_THROTTLE, MAX_THROTTLE),
        constrain(BASE_THROTTLE - outputY + outputZ, MIN_THROTTLE, MAX_THROTTLE),
        constrain(BASE_THROTTLE + outputY - outputZ, MIN_THROTTLE, MAX_THROTTLE),
        constrain(BASE_THROTTLE + outputY + outputZ, MIN_THROTTLE, MAX_THROTTLE)
    );
}

void loop() {
    server.handleClient();
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    float angleY = g.gyro.y * 57.2958;
    float angleZ = g.gyro.z * 57.2958;
    
    PIDOutput motorSpeeds = calculatePID(angleY, angleZ);
    
    // Print all data in a single line with comma separation
    Serial.print("Y:");
    Serial.print(angleY, 2);  // 2 decimal places
    Serial.print(",Z:");
    Serial.print(angleZ, 2);
    Serial.print(",FR:");
    Serial.print(motorSpeeds.frontRight);
    Serial.print(",FL:");
    Serial.print(motorSpeeds.frontLeft);
    Serial.print(",BR:");
    Serial.print(motorSpeeds.backRight);
    Serial.print(",BL:");
    Serial.println(motorSpeeds.backLeft);
    
    motor_FR.writeMicroseconds(motorSpeeds.frontRight);
    motor_FL.writeMicroseconds(motorSpeeds.frontLeft);
    motor_BR.writeMicroseconds(motorSpeeds.backRight);
    motor_BL.writeMicroseconds(motorSpeeds.backLeft);
    
    delay(10);
}