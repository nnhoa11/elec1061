#include <Servo.h>

// Pin assignments for IR sensors and servos
const int leftIrLedPin = 10;
const int leftIrReceiverPin = 11;
const int centerIrLedPin = 6;
const int centerIrReceiverPin = 7;
const int rightIrLedPin = 2;
const int rightIrReceiverPin = 3;
const int redLeftLedPin = A2;
const int redCenterLedPin = A1;
const int redRightLedPin = A0;
const int leftServoPin = 13;   // Pin for left servo
const int rightServoPin = 12;  // Pin for right servo

Servo servoLeft, servoRight;

// IR Object Detection Function
int irDetect(int irLedPin, int irReceiverPin, long frequency) {
  tone(irLedPin, frequency);          // Turn on the IR LED square wave
  delay(1);                           // Wait 1 ms
  noTone(irLedPin);                   // Turn off the IR LED
  int ir = digitalRead(irReceiverPin); // Read IR receiver signal (0 = detect, 1 = no detect)
  delay(1);                           // Small delay
  return ir;                          // Return detection (0 = detect, 1 = no detect)
}

// Servo movement functions
void moveForward() {
  servoLeft.writeMicroseconds(1600);   // Move forward (calibrated value)
  servoRight.writeMicroseconds(1300);  // Move forward (calibrated value)
}

void moveBackward() {
  servoLeft.writeMicroseconds(1300);   // Move backward (calibrated value)
  servoRight.writeMicroseconds(1600);  // Move backward (calibrated value)
}

void turnLeft() {
  servoLeft.writeMicroseconds(1400);   // Stop left wheel
  servoRight.writeMicroseconds(1600);  // Turn right wheel forward
  delay(300);                          // Adjust this delay for a smoother 90-degree turn
}

void turnRight() {
  servoLeft.writeMicroseconds(1600);   // Move left wheel forward for turn
  servoRight.writeMicroseconds(1400);  // Stop right wheel
  delay(300);                          // Adjust this delay for a smoother 90-degree turn
}

void stopMovement() {
  servoLeft.writeMicroseconds(1495);   // Stop left wheel
  servoRight.writeMicroseconds(1490);  // Stop right wheel
}

void setup() {
  // IR sensor pins
  pinMode(leftIrReceiverPin, INPUT);
  pinMode(leftIrLedPin, OUTPUT);
  pinMode(centerIrReceiverPin, INPUT);
  pinMode(centerIrLedPin, OUTPUT);
  pinMode(rightIrReceiverPin, INPUT);
  pinMode(rightIrLedPin, OUTPUT);

  // Red LED pins
  pinMode(redLeftLedPin, OUTPUT);
  pinMode(redCenterLedPin, OUTPUT);
  pinMode(redRightLedPin, OUTPUT);

  // Servo pins
  servoRight.attach(rightServoPin);
  servoLeft.attach(leftServoPin);

  Serial.begin(9600);

  // 5-second delay before starting
  // delay(5000);
}

void loop() {
  // Read left, center, and right IR sensors (38kHz frequency)
  int leftIrVal = irDetect(leftIrLedPin, leftIrReceiverPin, 38000);
  int centerIrVal = irDetect(centerIrLedPin, centerIrReceiverPin, 38000);
  int rightIrVal = irDetect(rightIrLedPin, rightIrReceiverPin, 38000);
  
  // Sensor output to serial for debugging
  Serial.print("Left: ");
  Serial.print(leftIrVal);
  Serial.print(" | Center: ");
  Serial.print(centerIrVal);
  Serial.print(" | Right: ");
  Serial.println(rightIrVal);

  // Update LED states based on detection
  digitalWrite(redLeftLedPin, leftIrVal == 0 ? HIGH : LOW);
  digitalWrite(redCenterLedPin, centerIrVal == 0 ? HIGH : LOW);
  digitalWrite(redRightLedPin, rightIrVal == 0 ? HIGH : LOW);

  // Decision logic for movement based on sensor readings
  if (centerIrVal == 0) {
    // Obstacle detected in front, stop and decide on turn
    stopMovement();
    Serial.println("Obstacle in front.");

    if (leftIrVal == 1 && rightIrVal == 0) {
      // If left is open and right is blocked, turn left
      Serial.println("Turning left.");
      turnLeft();
    } else if (rightIrVal == 1 && leftIrVal == 0) {
      // If right is open and left is blocked, turn right
      Serial.println("Turning right.");
      turnRight();
    } else if (leftIrVal == 1 && rightIrVal == 1) {
      // Both left and right open, randomly pick a direction or go forward
      Serial.println("Both sides open, moving forward.");
      moveForward();
    } else {
      // If no clear path, back up slightly and reassess
      Serial.println("Backing up.");
      moveBackward();
      delay(500);  // Move backward for a short time
    }
  } else {
    // No obstacle in front, continue moving forward
    Serial.println("Moving forward.");
    moveForward();
  }

  delay(100);  // Delay to avoid rapid sensor reading fluctuations
}
