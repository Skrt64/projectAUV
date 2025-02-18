#include <ESP32Servo.h>

#define RED_PIN 25
#define GREEN_PIN 26
#define BLUE_PIN 27

String receivedData;
int EX = 0, EY = 0;
int prevEX = -1, prevEY = -1;
int mappedValue = 0;

const int motor1Pin = 13;
const int motor2Pin = 12;

Servo motor1;
Servo motor2;

const int CHECK_INTERVAL = 500;
const int CHECK_COUNT = 6;
unsigned long lastCheckTime = 0;
int stableCount = 0;
int mode = 2;
bool isBlinkOn = false; // ใช้สำหรับกระพริบไฟ

void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 Ready! Send error_x,error_y to test.");
    
    pinMode(RED_PIN, OUTPUT);
    pinMode(GREEN_PIN, OUTPUT);
    pinMode(BLUE_PIN, OUTPUT);
    
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    motor1.setPeriodHertz(50);
    motor2.setPeriodHertz(50);

    motor1.attach(motor1Pin, 1000, 2000);
    motor2.attach(motor2Pin, 1000, 2000);

    motor1.writeMicroseconds(1505);
    motor2.writeMicroseconds(1505);

    delay(2000);
}

void setColor(int red, int green, int blue) {
    analogWrite(RED_PIN, red);
    analogWrite(GREEN_PIN, green);
    analogWrite(BLUE_PIN, blue);
}

void loop() {
    if (Serial.available() > 0) {
        receivedData = Serial.readStringUntil('\n');
        receivedData.trim();
        parseErrorData(receivedData);
    }

if (millis() - lastCheckTime >= CHECK_INTERVAL) {
        lastCheckTime = millis();    

        if (EX == 999) {
            // ไฟเป็นสีเหลืองก่อนเริ่มเช็ค
            if (stableCount == 0) {
                setColor(255, 255, 0);
            }

            // ถ้าค่าคงที่ไม่เปลี่ยน
            if (EX == prevEX && EY == prevEY) {
                stableCount++;

                // กระพริบไฟสีเหลืองระหว่างรอ 3 วินาที
                if (stableCount < CHECK_COUNT) {
                    isBlinkOn = !isBlinkOn;
                    if (isBlinkOn) {
                        setColor(255, 255, 0);
                    } else {
                        setColor(0, 0, 0);
                    }
                }
            } else {
                stableCount = 0;
                setColor(255, 255, 0); // ถ้ามีการเปลี่ยนแปลง รีเซ็ตไฟเป็นสีเหลือง
            }

            prevEX = EX;
            prevEY = EY;

            if (stableCount >= CHECK_COUNT) {
                mode = EY;
                if (mode == 1) {
                    setColor(0, 255, 0); // ไฟเขียว
                } else if (mode == 2) {
                    setColor(255, 0, 0); // ไฟแดง
                }
            }
        } else {
            stableCount = 0; // ถ้า EX ไม่ใช่ 999 ให้รีเซ็ต
            if (mode == 1) {
                    setColor(0, 255, 0); // ไฟเขียว
                } else if (mode == 2) {
                    setColor(255, 0, 0); // ไฟแดง
                }
              }   
    }

    // ควบคุมมอเตอร์
    if (mode == 1) {
        mappedValue = map(EY, 120, -120, 1000, 2000);
        mappedValue = constrain(mappedValue, 1000, 2000);
        motor1.writeMicroseconds(mappedValue);
        motor2.writeMicroseconds(mappedValue);
    } else if (mode == 2) {
        
        motor1.writeMicroseconds(1505);
        motor2.writeMicroseconds(1505);
    }
}

void parseErrorData(String data) {
    int commaIndex = data.indexOf(',');
    if (commaIndex > 0) {
        String exStr = data.substring(0, commaIndex);
        String eyStr = data.substring(commaIndex + 1);
        EX = exStr.toInt();
        EY = eyStr.toInt();
    } else {
        Serial.println("Invalid data format! Use format: error_x,error_y");
    }
}
