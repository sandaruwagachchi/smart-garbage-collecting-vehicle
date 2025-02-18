#include <Servo.h>

// Declare servo objects for Base, Shoulder, Elbow, and Gripper
Servo Base;
Servo Shoulder;
Servo Elbow;
Servo Gripper;

// Motor control pins
#define enA 13      // Enable1 L298 Pin enA 
#define in1 12      // Motor1  L298 Pin in1 
#define in2 11      // Motor1  L298 Pin in2 
#define in3 10      // Motor2  L298 Pin in3 
#define in4 9       // Motor2  L298 Pin in4 
#define enB 8       // Enable2 L298 Pin enB 

#define R_S A0      // IR sensor Right
#define L_S A1      // IR sensor Left

#define S0 2        // Color sensor pins
#define S1 3
#define S2 4
#define S3 5
#define sensorOut 6

// Adjustable Speeds
int baseSpeed = 60;
int turningSpeed = 100;

// Color Calibration Values (You MUST calibrate these!)
int green_min = 50;   // Minimum green pulse width (calibrate!)
int green_max = 100;  // Maximum green pulse width (calibrate!)

// Variables
int greenPW = 0;
bool isGreenDetected = false;
unsigned long greenDetectedTime = 0;
const unsigned long greenDelayStop = 250000; // 250 seconds stop delay
const unsigned long greenDelayForward = 2000; // 5 seconds forward delay
bool isForwardingAfterGreen = false;
unsigned long forwardStartTime = 0;

void setup() {
  // Attach servos to the respective pins
  Base.attach(17);      // Connect the Base servo motor to pin 17
  Shoulder.attach(16);  // Connect the Shoulder servo motor to pin 16
  Elbow.attach(15);     // Connect the Elbow servo motor to pin 15
  Gripper.attach(14);   // Connect the Gripper servo motor to pin 14

  // Motor pins
  pinMode(enA, OUTPUT); 
  pinMode(in1, OUTPUT); 
  pinMode(in2, OUTPUT); 
  pinMode(in3, OUTPUT);  
  pinMode(in4, OUTPUT); 
  pinMode(enB, OUTPUT); 

  // IR sensor pins
  pinMode(R_S, INPUT); 
  pinMode(L_S, INPUT); 

  // Color sensor pins
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);

  // Set pulse width scaling for color sensor (adjust as needed)
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  Serial.begin(9600);
  Serial.println("Line Following with Green Detection");
}

void loop() {
  // Check for green
  greenPW = getGreenPW();
  if (greenPW >= green_min && greenPW <= green_max) {
    if (!isGreenDetected) { // Only trigger once per green detection
      Serial.println("Green Detected!");
      isGreenDetected = true;
      greenDetectedTime = millis(); // Record the time green was detected
      Stop(); // Stop immediately upon detecting green
      isForwardingAfterGreen = false; // Reset forwarding flag

      // Start the robotic arm sequence
      roboticArmSequence();
    }
  }

  // Check if green stop delay is over
  if (isGreenDetected && (millis() - greenDetectedTime >= greenDelayStop) && !isForwardingAfterGreen) {
    isForwardingAfterGreen = true;
    forwardStartTime = millis();
    forward(); // Start moving forward
    Serial.println("Moving forward after green");
  }

  // Check if forward delay is over
  if (isForwardingAfterGreen && (millis() - forwardStartTime >= greenDelayForward)) {
    isGreenDetected = false; // Reset the main flag
    isForwardingAfterGreen = false; // Reset forwarding flag
    Serial.println("Resuming Line Following...");
  }

  // Line following logic (only if green is not currently being handled)
  if (!isGreenDetected) { 
    if ((digitalRead(R_S) == 0) && (digitalRead(L_S) == 0)) {
      forward();
    } else if ((digitalRead(R_S) == 1) && (digitalRead(L_S) == 0)) {
      turnRight();
    } else if ((digitalRead(R_S) == 0) && (digitalRead(L_S) == 1)) {
      turnLeft();
    } else if ((digitalRead(R_S) == 1) && (digitalRead(L_S) == 1)) {
      Stop();
    }
  }
}

// Motor control functions
void forward() {
  analogWrite(enA, baseSpeed); // Motor1 speed
  analogWrite(enB, baseSpeed); // Motor2 speed
  digitalWrite(in1, HIGH); 
  digitalWrite(in2, LOW);  
  digitalWrite(in3, LOW);  
  digitalWrite(in4, HIGH); 
}

void turnRight() {
  analogWrite(enA, turningSpeed); 
  analogWrite(enB, turningSpeed);
  digitalWrite(in1, LOW);  
  digitalWrite(in2, HIGH); 
  digitalWrite(in3, LOW);  
  digitalWrite(in4, HIGH); 
}

void turnLeft() {
  analogWrite(enA, turningSpeed); 
  analogWrite(enB, turningSpeed);
  digitalWrite(in1, HIGH); 
  digitalWrite(in2, LOW);  
  digitalWrite(in3, HIGH); 
  digitalWrite(in4, LOW);  
}

void Stop() {
  analogWrite(enA, 0); // Motor1 speed - Stop
  analogWrite(enB, 0); // Motor2 speed - Stop
  digitalWrite(in1, LOW); 
  digitalWrite(in2, LOW);  
  digitalWrite(in3, LOW);  
  digitalWrite(in4, LOW); 
}

// Color sensor function
int getGreenPW() {
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  return pulseIn(sensorOut, LOW);
}

// Robotic arm sequence
void roboticArmSequence() {
  // Move Elbow
  for (int i = 130; i <= 130; i++) { 
    Elbow.write(i); // Move Elbow servo back to the previous position
    delay(100);      // Smooth movement with 50ms delay
  }
  delay(5000); 
   // --- Step 2: Shoulder Movement ---
  for (int i = 90; i <= 90; i++) { 
    Shoulder.write(i); // Move Shoulder servo to the next position
    delay(100);         // Smooth movement with 50ms delay
  }
   delay(5000);


 for (int i = 0; i <= 0; i++) { 
    Gripper.write(i); // open Gripper
    delay(100);        // Smooth movement with 50ms delay
  }
  delay(5000);

  // --- Step 1: Base Movement ---
  for (int i = 10; i <= 120; i++) { 
    Base.write(i);  // Move Base servo to the next position
    delay(100);      // Smooth movement with 50ms delay
  }
  delay(5000); // Pause for 3 seconds at 120 degrees

   for (int i = 130; i >= 50; i--) { 
    Elbow.write(i); // Move Elbow servo back to the previous position
    delay(100);      // Smooth movement with 50ms delay
  }
  delay(5000); 

for (int i = 0; i <= 180; i++) { 
    Gripper.write(i); // open Gripper
    delay(100);        // Smooth movement with 50ms delay
  }
  delay(5000);

   // --- Step 2: Shoulder Movement ---
  for (int i = 90; i <= 160; i++) { 
    Shoulder.write(i); // Move Shoulder servo to the next position
    delay(100);         // Smooth movement with 50ms delay
  }
   delay(5000);

   for (int i = 180; i >= 0; i--) { 
    Gripper.write(i); // Close Gripper
    delay(100);        // Smooth movement with 50ms delay
  }
  delay(3000);


 // Move Shoulder
   for (int i = 160; i >= 90; i--) { 
    Shoulder.write(i); // Move Shoulder servo to the next position
    delay(100);         // Smooth movement with 50ms delay
  }
   delay(5000); 

 for (int i = 50; i <= 130; i++) { 
    Elbow.write(i); // Move Elbow servo back to the previous position
    delay(100);      // Smooth movement with 50ms delay
  }
  delay(5000); 


  for (int i = 120; i >= 10; i--) { 
    Base.write(i);  // Move Base servo back to the previous position
    delay(100);      // Smooth movement with 50ms delay
  }
  delay(5000);

 for (int i = 10; i <= 120; i++) { 
    Base.write(i);  // Move Base servo back to the previous position
    delay(100);      // Smooth movement with 50ms delay
  }
  delay(5000);


   for (int i = 130; i >= 50; i--) { 
    Elbow.write(i); // Move Elbow servo back to the previous position
    delay(100);      // Smooth movement with 50ms delay
  }
  delay(5000); 

  for (int i = 90; i <= 160; i++) { 
    Shoulder.write(i); // Move Shoulder servo to the next position
    delay(100);         // Smooth movement with 50ms delay
  }
   delay(5000);

   for (int i = 0; i <= 180; i++) { 
    Gripper.write(i); // open Gripper
    delay(100);        // Smooth movement with 50ms delay
  }
  delay(5000);

// Move Shoulder
   for (int i = 160; i >= 90; i--) { 
    Shoulder.write(i); // Move Shoulder servo to the next position
    delay(100);         // Smooth movement with 50ms delay
  }
   delay(5000); 
    for (int i = 180; i >= 0; i--) { 
    Gripper.write(i); // Close Gripper
    delay(100);        // Smooth movement with 50ms delay
  }
  delay(3000);

  for (int i = 50; i <= 130; i++) { 
    Elbow.write(i); // Move Elbow servo back to the previous position
    delay(100);      // Smooth movement with 50ms delay
  }
  delay(5000); 

   for (int i = 120; i >= 10; i--) { 
    Base.write(i);  // Move Base servo back to the previous position
    delay(100);      // Smooth movement with 50ms delay
  }
  delay(5000);








  
}