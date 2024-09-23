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
int prevLeft = 5;
int prevRight = 5;

Servo servoLeft, servoRight;

int irDistance(int irLedPin, int irReceiverPin)
{
   int distance = 0;
   for(long f = 38000; f <= 42000; f += 1000)
   {
      distance += irDetect(irLedPin, irReceiverPin, f);
   }
   return distance;
}
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
void moveForward(int leftSpeed, int rightSpeed) {
  servoLeft.writeMicroseconds(leftSpeed);   // Adjust left servo speed
  servoRight.writeMicroseconds(rightSpeed);  // Adjust right servo speed
}

void turnLeft() {
  servoLeft.writeMicroseconds(1400);    // Stop left wheel
  servoRight.writeMicroseconds(1400);   // Move right wheel forward for turn
  delay(600);                        // Adjust this delay for 90-degree turn

}

void turnRight() {
  servoLeft.writeMicroseconds( 1600);    // Move left wheel forward for turn
  servoRight.writeMicroseconds( 1600);   // Stop right wheel
  delay(600);                        // Adjust this delay for 90-degree turn

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
  stopMovement();
  prevLeft = irDistance(leftIrLedPin, leftIrReceiverPin);
  prevRight = irDistance(rightIrLedPin, rightIrReceiverPin);
   delay(5000);
}

void loop() {
  // Read left, center, and right IR sensors (38kHz frequency)
  int leftIrVal = irDistance(leftIrLedPin, leftIrReceiverPin);
  int centerIrVal = irDistance(centerIrLedPin, centerIrReceiverPin);
  int rightIrVal = irDistance(rightIrLedPin, rightIrReceiverPin);
  
  // Sensor output to serial for debugging
  Serial.print("Left: ");
  Serial.print(leftIrVal);
  Serial.print(" | Center: ");
  Serial.print(centerIrVal);
  Serial.print(" | Right: ");
  Serial.println(rightIrVal);
  Serial.print("preLeft: ");
  Serial.print(prevLeft);
  Serial.print(" | prevRight: ");
  Serial.println(prevRight);

  // Update LED states based on detection
  digitalWrite(redLeftLedPin, leftIrVal < 5 ? HIGH : LOW);
  digitalWrite(redCenterLedPin, centerIrVal < 5 ? HIGH : LOW);
  digitalWrite(redRightLedPin, rightIrVal < 5  ? HIGH : LOW);

  // Decision logic for movement based on sensor readings
  if (leftIrVal < 5 && rightIrVal < 5 && centerIrVal < 5){
    stopMovement();
    Serial.print('Stopping');
  }
  else
  if (centerIrVal < 5) {
    // Obstacle detected in front, stop and decide on turn
    stopMovement();
    Serial.println("Obstacle in front.");
    if (rightIrVal < 5 && leftIrVal > 4) {
      // If left is open and right is blocked, turn left
      Serial.println("Turning left.");
      turnLeft();
    } else if (leftIrVal < 5 && rightIrVal > 4) {
      // If right is open and left is blocked, turn right
      Serial.println("Turning right.");
      turnRight();
    }
    moveForward(1600, 1400); 
  } 
  else {
    // No obstacle in front, correct slight angles based on left/right IR sensors
    if (prevLeft - leftIrVal < 0) {
      // Slight drift to the left, so turn right slightly
      Serial.println("Correcting right.");
      moveForward(1550, 1400);  // Slow down left wheel to correct the path
    } else if (prevRight - rightIrVal > 0) {
      // Slight drift to the right, so turn left slightly
      Serial.println("Correcting left.");
      moveForward(1600, 1450);  // Slow down right wheel to correct the path
    } else {
      // Move forward if both sides are balanced
      Serial.println("Moving forward.");
      moveForward(1600, 1400);  // Balanced forward movement
    }
  }

  delay(100);  // Delay to avoid rapid sensor reading fluctuations
}
