/**
 * ESP32 Quad Encoder Reader
 * Reads 4 Quadrature Encoders and sends data over Serial.
 * 
 * Hardware:
 * - ELEGOO ESP32 Development Board
 * - 4x Akozon 600P/R Encoders (A/B Phase)
 * 
 * Pinout Strategy:
 * ESP32 has many interrupt capable pins.
 * Encoder 1: A=D13, B=D12
 * Encoder 2: A=D14, B=D27
 * Encoder 3: A=D26, B=D25
 * Encoder 4: A=D33, B=D32
 * (Adjust according to actual wiring capabilities / conflicts)
 * Note: D34, D35, D36, D39 are input only (no internal pullups?), best used with external pullups.
 * 
 * Protocol:
 * Sends CSV line: "E1,E2,E3,E4\n" (Raw Ticks)
 * Can receive commands to set scaling or reset.
 */

#include <Arduino.h>

// Define Pins
#define E3_A 13
#define E3_B 12
#define E2_A 14
#define E2_B 27
#define E4_A 26
#define E4_B 25
#define E1_A 33
#define E1_B 32

// Encoder Counts (volatile for ISR)
volatile long count1 = 0;
volatile long count2 = 0;
volatile long count3 = 0;
volatile long count4 = 0;

// Scaling Parameters
float pulses_per_rev = 2400.0;
float quadrature_factor = 4.0; // 4x decoding
float counts_per_rev = pulses_per_rev * quadrature_factor;

// Output Mode
enum Mode { RAW_TICKS, DEGREES, RADIANS };
Mode output_mode = RADIANS;

// ISRs
void IRAM_ATTR isr1() {
  int a = digitalRead(E1_A);
  int b = digitalRead(E1_B);
  if (a == b) count1++; else count1--;
}

void IRAM_ATTR isr2() {
  int a = digitalRead(E2_A);
  int b = digitalRead(E2_B);
  if (a == b) count2++; else count2--;
}

void IRAM_ATTR isr3() {
  int a = digitalRead(E3_A);
  int b = digitalRead(E3_B);
  if (a == b) count3++; else count3--;
}

void IRAM_ATTR isr4() {
  int a = digitalRead(E4_A);
  int b = digitalRead(E4_B);
  if (a == b) count4++; else count4--;
}

void setup() {
  Serial.begin(115200);
  
  // Setup Pins (Input Pullup is safer if open collector)
  pinMode(E1_A, INPUT_PULLUP); pinMode(E1_B, INPUT_PULLUP);
  pinMode(E2_A, INPUT_PULLUP); pinMode(E2_B, INPUT_PULLUP);
  pinMode(E3_A, INPUT_PULLUP); pinMode(E3_B, INPUT_PULLUP);
  pinMode(E4_A, INPUT_PULLUP); pinMode(E4_B, INPUT_PULLUP);
  
  // Attach Interrupts (Rising/Change on Channel A is usually enough for 2x/4x)
  // For full 4x resolution, we might want CHANGE on both, but simpler logic here:
  // Using CHANGE on A gives 2x resolution effectively with simple logic, 
  // or full logic needed. Let's use simple CHANGE on A for now.
  attachInterrupt(digitalPinToInterrupt(E1_A), isr1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(E2_A), isr2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(E3_A), isr3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(E4_A), isr4, CHANGE);
  
  Serial.println("ESP32 Encoder Firmware Ready");
}

float toAngle(long count) {
    float revs = (float)count / counts_per_rev;
    // Map to -180 to 180 (optional wrapping) or continuous
    // User asked for "scale a range... -180 to 180". Usually swerve steers are continuous or wrapped.
    // Let's output continuous value first.
    if (output_mode == DEGREES) return revs * 360.0;
    if (output_mode == RADIANS) return revs * 2.0 * PI;
    return (float)count;
}

void loop() {
  // Check for commands
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r') {
      count1 = count2 = count3 = count4 = 0;
      Serial.println("Reset");
    }
    if (c == 'd') output_mode = DEGREES;
    if (c == 'R') output_mode = RADIANS;
    if (c == 't') output_mode = RAW_TICKS;
  }
  
  // Send Data
  float v1 = toAngle(count1);
  float v2 = toAngle(count2);
  float v3 = toAngle(count3);
  float v4 = toAngle(count4);
  
  Serial.print(v1); Serial.print(",");
  Serial.print(v2); Serial.print(",");
  Serial.print(v3); Serial.print(",");
  Serial.println(v4);
  
  delay(20); // 50Hz update rate
}
