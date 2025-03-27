#include "src/SBUS/sbus.h"

#define ESC1_PIN 5   // Motor 1
#define ESC2_PIN 6   // Motor 2
#define ESC3_PIN 9   // Motor 3
#define ESC4_PIN 10  // Motor 4

bfs::SbusRx sbus_rx(&Serial2);
bfs::SbusData data;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  sbus_rx.Begin();

  pinMode(ESC1_PIN, OUTPUT);
  pinMode(ESC2_PIN, OUTPUT);
  pinMode(ESC3_PIN, OUTPUT);
  pinMode(ESC4_PIN, OUTPUT);
}

void loop() {
  if (sbus_rx.Read()) {
    data = sbus_rx.data();

    // Read throttle, roll, pitch, yaw (assume CH1–CH4)
    int throttle = data.ch[2];  // CH3 (Throttle)
    int roll = data.ch[0];      // CH1
    int pitch = data.ch[1];     // CH2
    int yaw = data.ch[3];       // CH4

    // Map SBUS values (172-1811) to Oneshot125 range (125-250µs)
    int esc1_pulse = map(throttle + roll - pitch + yaw, 172, 1811, 125, 250);
    int esc2_pulse = map(throttle - roll - pitch - yaw, 172, 1811, 125, 250);
    int esc3_pulse = map(throttle - roll + pitch + yaw, 172, 1811, 125, 250);
    int esc4_pulse = map(throttle + roll + pitch - yaw, 172, 1811, 125, 250);

    // Ensure values are within limits
    esc1_pulse = constrain(esc1_pulse, 125, 250);
    esc2_pulse = constrain(esc2_pulse, 125, 250);
    esc3_pulse = constrain(esc3_pulse, 125, 250);
    esc4_pulse = constrain(esc4_pulse, 125, 250);

    // Send Oneshot125 pulses
    sendOneshot125(ESC1_PIN, esc1_pulse);
    sendOneshot125(ESC2_PIN, esc2_pulse);
    sendOneshot125(ESC3_PIN, esc3_pulse);
    sendOneshot125(ESC4_PIN, esc4_pulse);

    // Debugging output
    Serial.print("Throttle: ");
    Serial.print(throttle);
    Serial.print(" | ESC1: ");
    Serial.print(esc1_pulse);
    Serial.print(" | ESC2: ");
    Serial.print(esc2_pulse);
    Serial.print(" | ESC3: ");
    Serial.print(esc3_pulse);
    Serial.print(" | ESC4: ");
    Serial.println(esc4_pulse);
  }

  delay(10);  // Small delay for smooth signal
}

// Function to generate Oneshot125 pulse
void sendOneshot125(int pin, int pulseWidth) {
  digitalWrite(pin, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(pin, LOW);
}
