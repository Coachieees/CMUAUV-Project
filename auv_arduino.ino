#include <Servo.h>
#include <Adafruit_NeoPixel.h>

#define LED_PIN 22
#define NUM_LEDS 8
Adafruit_NeoPixel pixels(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

Servo thrusters[8];

// 🟢 อัปเดตพินตามคำขอ: T1-T4=4-7, T5-T8=9-12 (เว้นพิน 8 ไว้ไม่เบียดกัน)
int thrusterPins[8] = {4, 5, 6, 7, 9, 10, 11, 12}; 

int currentPWM[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
bool isConnected = false;
const int DEADBAND = 25;

// ตัวแปรควบคุมไฟ
bool showThrusterLEDs = true; 
bool was_led_on = true; // จำสถานะว่าไฟเพิ่งเปิดอยู่หรือไม่ (สำหรับโหมด MPC)

// ตัวแปร Safety Watchdog
unsigned long last_receive_time = 0;
const unsigned long TIMEOUT_MS = 2000;
unsigned long last_blink_time = 0;
bool red_led_state = false;

void setup() {
  Serial.begin(115200);
  pixels.begin();
  pixels.clear();
  pixels.show();

  for(int i = 0; i < 8; i++) {
    thrusters[i].attach(thrusterPins[i]);
    thrusters[i].writeMicroseconds(1500);
  }

  blinkRed(); 
  last_receive_time = millis();
}

void blinkGreen() {
  for(int blink = 0; blink < 3; blink++) {
    for(int i = 0; i < 4; i++) { 
      pixels.setPixelColor(i, pixels.Color(0, 255, 0));
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
    for(int i = 0; i < 4; i++) { 
      pixels.setPixelColor(i, pixels.Color(255, 85, 0));
    }
    pixels.show();
    delay(200);
    pixels.clear();
    pixels.show();
    delay(200);
  }
}

void stopAllThrusters() {
  for(int i = 0; i < 8; i++) {
    thrusters[i].writeMicroseconds(1500);
    currentPWM[i] = 1500;
  }
  pixels.clear();
  pixels.show();
}

void blinkRedContinuously() {
  if (millis() - last_blink_time > 250) {
    last_blink_time = millis();
    red_led_state = !red_led_state;

    if (red_led_state) {
      for(int i = 0; i < 4; i++) { // กระพริบแค่ 4 ดวงแรก
        pixels.setPixelColor(i, pixels.Color(255, 0, 0));
      }
    } else {
      pixels.clear();
    }
    pixels.show();
  }
}

void loop() {
  if (Serial.available()) {
    String msg = Serial.readStringUntil('\n');
    msg.trim();
    
    if (msg.startsWith("PWM,")) {
      
      if (!isConnected) {
        blinkGreen();
        isConnected = true;
      }
      
      int commaIndex = msg.indexOf(',');
      int parsedValues = 0;
      int tempValues[9]; 

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

      // 🟢 ตรวจสอบความครบถ้วนของข้อมูล
      if (parsedValues >= 9) {
        
        last_receive_time = millis(); 
        
        showThrusterLEDs = (tempValues[8] == 1);

        // ==============================================
        // โหมดปกติ: เปิดไฟสถานะ (อาจมีดีเลย์เล็กน้อยจาก NeoPixel)
        // ==============================================
        if (showThrusterLEDs) {
          pixels.clear(); 
          
          for (int i = 0; i < 8; i++) {
            // ใช้ constrain ป้องกันค่าขยะ (Garbage Data)
            int pwm = constrain(tempValues[i], 1000, 2000); 
            currentPWM[i] = pwm;
            thrusters[i].writeMicroseconds(pwm);

            if (pwm > 1500 + DEADBAND) {
              pixels.setPixelColor(i, pixels.Color(255, 0, 0));
            } else if (pwm < 1500 - DEADBAND) {
              pixels.setPixelColor(i, pixels.Color(0, 0, 255));
            }
          }
          pixels.show(); 
          was_led_on = true; 
        } 
        // ==============================================
        // โหมด MPC: ปิดไฟสถานะ (ไม่เรียก pixels.show เพื่อปลดล็อกสปีด 100%)
        // ==============================================
        else {
          for (int i = 0; i < 8; i++) {
            int pwm = constrain(tempValues[i], 1000, 2000); 
            currentPWM[i] = pwm;
            thrusters[i].writeMicroseconds(pwm);
          }

          // ปิดไฟแค่รอบแรกที่โดนสั่งปิด เพื่อไม่ให้ Arduino หน่วงเวลาซ้ำๆ
          if (was_led_on) {
            pixels.clear();
            pixels.show(); 
            was_led_on = false; 
          }
        }
      }
    }
  }

  // ระบบ Watchdog
  if (millis() - last_receive_time > TIMEOUT_MS) {
    if (isConnected) {
      isConnected = false;
      stopAllThrusters();
    }
    blinkRedContinuously();
  }
}
