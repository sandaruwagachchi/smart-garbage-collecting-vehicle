#include <Servo.h>

#define TRIG_PIN 12
#define ECHO_PIN 11
#define SERVO_PIN 9

Servo servo;

void setup() {
  Serial.begin(9600);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  servo.attach(SERVO_PIN);
  servo.write(50); // Initial position
}

void loop() {
  long duration;
  float distance;

  // Send ultrasonic pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read echo response
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = (duration * 0.0343) / 2; // Convert to cm

  // Servo Control based on distance
  if (distance > 0 && distance <= 5) { // Changed condition to <= 5
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm. Object detected! Moving Servo from 50° to 180°");

    // Move servo from 50° to 180°
    for (int angle = 50; angle <= 180; angle++) {
      servo.write(angle);
      delay(20); 
    }

    delay(1500);

    // Move servo back from 180° to 50°
    for (int angle = 180; angle >= 50; angle--) {
      servo.write(angle);
      delay(20); 
    }
  } else {
    servo.write(50); // Reset Servo position
  }

  delay(500);
}