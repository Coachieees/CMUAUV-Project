#include <Servo.h>

Servo thrusters[8];

// 🟢 พินมอเตอร์: T1-T4=4-7, T5-T8=9-12 (เว้นพิน 8)
int thrusterPins[8] = {4, 5, 6, 7, 9, 10, 11, 12}; 

int currentPWM[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
bool isConnected = false;

// ตัวแปร Safety Watchdog
unsigned long last_receive_time = 0;
const unsigned long TIMEOUT_MS = 2000;

void setup() {
  Serial.begin(115200);

  for(int i = 0; i < 8; i++) {
    thrusters[i].attach(thrusterPins[i]);
    thrusters[i].writeMicroseconds(1500);
  }

  last_receive_time = millis();
}

void stopAllThrusters() {
  for(int i = 0; i < 8; i++) {
    thrusters[i].writeMicroseconds(1500);
    currentPWM[i] = 1500;
  }
}

void loop() {
  if (Serial.available()) {
    String msg = Serial.readStringUntil('\n');
    msg.trim();
    
    if (msg.startsWith("PWM,")) {
      
      if (!isConnected) {
        isConnected = true;
      }
      
      int commaIndex = msg.indexOf(',');
      int parsedValues = 0;
      int tempValues[9]; // ยังคงเผื่อไว้รับ 9 ค่า (เพื่อรองรับข้อมูล LED จาก Dashboard เดิม)

      for (int i = 0; i < 9; i++) {
        if (commaIndex == -1) break;
        int nextComma = msg.indexOf(',', commaIndex + 1);
        String valStr;
        if (nextComma == -1) {
          valStr = msg.substring(commaIndex + 1);
        } else {
          valStr = msg.substring(commaIndex + 1, nextComma);
        }
        tempValues[i] = valStr.toInt();
        commaIndex = nextComma;
        parsedValues++;
      }

      // 🟢 ตรวจสอบความครบถ้วนของข้อมูล (เช็คแค่ 8 ค่าแรกที่เป็น PWM ก็พอ)
      if (parsedValues >= 8) {
        
        last_receive_time = millis(); 
        
        // สั่งงาน Thruster ทันที ไม่มีอะไรมาหน่วงเวลาแล้ว
        for (int i = 0; i < 8; i++) {
          int pwm = constrain(tempValues[i], 1000, 2000); 
          currentPWM[i] = pwm;
          thrusters[i].writeMicroseconds(pwm);
        }
      }
    }
  }

  // ระบบ Watchdog ตัดการทำงานถ้ารับข้อมูลไม่ได้เกิน 2 วินาที
  if (millis() - last_receive_time > TIMEOUT_MS) {
    if (isConnected) {
      isConnected = false;
      stopAllThrusters();
    }
  }
}
