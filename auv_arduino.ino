#include <Servo.h>
#include <Adafruit_NeoPixel.h>

// --- ตั้งค่า LED ---
#define LED_PIN 22           // เปลี่ยนให้ตรงกับขาที่ต่อ WS2812
#define NUM_LEDS 8
Adafruit_NeoPixel pixels(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// --- ตั้งค่า Thrusters ---
Servo thrusters[8];
// เปลี่ยนหมายเลข Pin ให้ตรงกับบอร์ด Arduino ของคุณ
int thrusterPins[8] = {5, 6, 7, 8, 9, 10, 11, 12}; 

// --- ตัวแปรระบบ ---
int currentPWM[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
bool isConnected = false;
const int DEADBAND = 25; // ค่า Offset (1500 +/- 25 คือค่า Neutral)

void setup() {
  Serial.begin(115200);
  
  // เริ่มการทำงาน LED (ปิดไฟทั้งหมดตอนเริ่มต้น)
  pixels.begin();
  pixels.clear();
  pixels.show();

  // ตั้งค่าเริ่มต้นให้ ESC (1500 = หยุดนิ่ง)
  for(int i = 0; i < 8; i++) {
    thrusters[i].attach(thrusterPins[i]);
    thrusters[i].writeMicroseconds(1500);
  }

  blinkRed();
}

// ฟังก์ชันกระพริบสีเขียว 3 ครั้งเมื่อเชื่อมต่อสำเร็จ
void blinkGreen() {
  for(int blink = 0; blink < 3; blink++) {
    for(int i = 0; i < NUM_LEDS; i++) {
      pixels.setPixelColor(i, pixels.Color(0, 255, 0)); // สีเขียว
    }
    pixels.show();
    delay(200);
    pixels.clear();
    pixels.show();
    delay(200);
  }
}

void blinkRed() {
  for(int blink = 0; blink < 3; blink++) {
    for(int i = 0; i < NUM_LEDS; i++) {
      pixels.setPixelColor(i, pixels.Color(255, 85, 0)); // สีแดง
    }
    pixels.show();
    delay(200);
    pixels.clear();
    pixels.show();
    delay(200);
  }
}

void loop() {
  if (Serial.available()) {
    String msg = Serial.readStringUntil('\n');
    msg.trim();
    
    // ตรวจสอบข้อมูลว่าขึ้นต้นด้วย "PWM," หรือไม่
    if (msg.startsWith("PWM,")) {
      
      // ถ้าเป็นการได้รับข้อมูลครั้งแรก ให้กระพริบไฟสีเขียว
      if (!isConnected) {
        blinkGreen();
        isConnected = true;
      }
      
      // แยกค่า PWM 8 ค่าออกมาจาก String "PWM,1500,1500,..."
      int commaIndex = msg.indexOf(',');
      for (int i = 0; i < 8; i++) {
        if (commaIndex == -1) break;
        int nextComma = msg.indexOf(',', commaIndex + 1);
        String valStr;
        if (nextComma == -1) {
          valStr = msg.substring(commaIndex + 1);
        } else {
          valStr = msg.substring(commaIndex + 1, nextComma);
        }
        currentPWM[i] = valStr.toInt();
        commaIndex = nextComma;
      }

      // สั่งงาน Thrusters และอัปเดตสี LED ตามเงื่อนไข
      for (int i = 0; i < 8; i++) {
        int pwm = currentPWM[i];
        
        // ส่งสัญญาณให้ ESC
        thrusters[i].writeMicroseconds(pwm);

        // จัดการ LED ตามค่า PWM
        if (pwm > 1500 + DEADBAND) {
          // Forward -> สีแดง
          pixels.setPixelColor(i, pixels.Color(255, 0, 0)); 
        } else if (pwm < 1500 - DEADBAND) {
          // Backward -> สีน้ำเงิน
          pixels.setPixelColor(i, pixels.Color(0, 0, 255)); 
        } else {
          // Neutral -> ปิดไฟ
          pixels.setPixelColor(i, pixels.Color(0, 0, 0)); 
        }
      }
      pixels.show(); // สั่งให้หลอดไฟอัปเดตสีพร้อมกัน
    }
  }
}