// Pin assignments
#include <Servo.h>
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
  int ir = digitalRead(irReceiverPin); // Read IR receiver signal
  delay(1);                           // Small delay
  return ir;                          // Return detection (0 = detect, 1 = no detect)
}

// Servo movement functions
void moveForward() {
  // Adjust the writeMicroseconds values based on your servo calibration
  servoLeft.writeMicroseconds(1600);    // Move forward (calibrated value)
  servoRight.writeMicroseconds(1300);   // Move forward (calibrated value)
}

void turnLeft() {
  servoLeft.writeMicroseconds(1400);    // Stop left wheel
  servoRight.writeMicroseconds(1400);   // Move right wheel forward for turn
  delay(250);                        // Adjust this delay for 90-degree turn
}

void turnRight() {
  servoLeft.writeMicroseconds( 1600);    // Move left wheel forward for turn
  servoRight.writeMicroseconds( 1600);   // Stop right wheel
  delay(250);                        // Adjust this delay for 90-degree turn
}

void stopMovement() {
  servoLeft.writeMicroseconds( 1495);    // Stop left wheel
  servoRight.writeMicroseconds(1490);   // Stop right wheel
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
  // Check left IR sensor (38kHz frequency)
  int leftIrVal = irDetect(leftIrLedPin, leftIrReceiverPin, 38000);
  if (leftIrVal == 0) {
    digitalWrite(redLeftLedPin, HIGH);  // Turn on left red LED if obstacle detected
    Serial.println("Left side detected opening.");
  } else {
    digitalWrite(redLeftLedPin, LOW);   // Turn off left red LED
  }

  // Check center IR sensor (38kHz frequency)
  
  // Check right IR sensor (38kHz frequency)
  int rightIrVal = irDetect(rightIrLedPin, rightIrReceiverPin, 38000);
  if (rightIrVal == 0) {
    digitalWrite(redRightLedPin, HIGH);   // Turn on right red LED if obstacle detected
    Serial.println("Right side detected opening.");
  } else {
    digitalWrite(redRightLedPin, LOW);    // Turn off right red LED
  }
int centerIrVal = irDetect(centerIrLedPin, centerIrReceiverPin, 38000);
  if (centerIrVal == 0) {
    digitalWrite(redCenterLedPin, HIGH);  // Turn on center red LED if obstacle detected
    Serial.println("Obstacle detected in front.");
    stopMovement();                       // Stop if obstacle is directly in front
    if (leftIrVal == 1) {                 // If no obstacle on the left, turn left
      turnLeft();
    } else if (rightIrVal == 1) {         // If no obstacle on the right, turn right
      turnRight();
    }
  } else {
    digitalWrite(redCenterLedPin, LOW);   // Turn off center red LED
  }

  // Move forward if no obstacle in front
  if (centerIrVal == 1) {
    moveForward();
  }

  delay(500);  // Adjust this value based on desired forward speed
}
