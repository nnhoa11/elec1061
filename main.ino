#include <Servo.h>

// IR Sensor Pins
const int leftIrLedPin = 10;
const int leftIrReceiverPin = 11;
const int leftIndicatorLedPin = A2;

const int midIrLedPin = 6;
const int midIrReceiverPin = 7;
const int midIndicatorLedPin = A1;

const int rightIrLedPin = 2;
const int rightIrReceiverPin = 3;
const int rightIndicatorLedPin = A0;

// Servo Pins
const int leftServoPin = 12;
const int rightServoPin = 13;

Servo leftServo;
Servo rightServo;

// Constants for servo control
const int SERVO_STOP = 1495;
const int SERVO_MAX_FORWARD = 1300;
const int SERVO_MAX_REVERSE = 1700;

// Thresholds for obstacle detection (tried 4233)
const int OBSTACLE_THRESHOLD = 4; 
const int CLOSE_OBSTACLE_THRESHOLD = 1; 
const int MID_OBSTACLE_THRESHOLD = 2;
const int DEAD_END_THRESHOLD = 4;

void setup() {
  pinMode(leftIrReceiverPin, INPUT);
  pinMode(leftIrLedPin, OUTPUT);
  pinMode(leftIndicatorLedPin, OUTPUT);

  pinMode(midIrReceiverPin, INPUT);
  pinMode(midIrLedPin, OUTPUT);
  pinMode(midIndicatorLedPin, OUTPUT);

  pinMode(rightIrReceiverPin, INPUT);
  pinMode(rightIrLedPin, OUTPUT);
  pinMode(rightIndicatorLedPin, OUTPUT);

  leftServo.attach(leftServoPin);
  rightServo.attach(rightServoPin);

  leftServo.writeMicroseconds(SERVO_STOP);
  rightServo.writeMicroseconds(SERVO_STOP);

  Serial.begin(9600);
}

void loop() {
  int leftDistance = irDistance(leftIrLedPin, leftIrReceiverPin);
  int midDistance = irDistance(midIrLedPin, midIrReceiverPin);
  int rightDistance = irDistance(rightIrLedPin, rightIrReceiverPin);

  Serial.print("Left: ");
  Serial.print(leftDistance);
  Serial.print("Mid: ");
  Serial.print(midDistance);
  Serial.print("Right: ");
  Serial.println(rightDistance);

  updateIndicatorLed(leftIndicatorLedPin, leftDistance);
  updateIndicatorLed(midIndicatorLedPin, midDistance);
  updateIndicatorLed(rightIndicatorLedPin, rightDistance);

  obstacleAvoidance(leftDistance, midDistance, rightDistance);

  delay(50);  // Delay before mext reading
}

void obstacleAvoidance(int left, int mid, int right) {
  if (left <= DEAD_END_THRESHOLD && mid <= DEAD_END_THRESHOLD && right <= DEAD_END_THRESHOLD) {
    // Dead end detected, stop the robot
   // moveRobot(0, 0);
      stopMovement();
    return;  // Exit the function to keep the robot stopped
  }

  if (mid <= CLOSE_OBSTACLE_THRESHOLD) {
    // Obstacle very close in front, reverse
    moveRobot(-50, -50);
    delay(100);
    
    // After reversing, turn in the direction with more space
    if (left > right) {
      moveRobot(40, -40);  // Turn left
    } else {
      moveRobot(-40, 40);  // Turn right
    }
    delay(300);
  } else if (mid <= MID_OBSTACLE_THRESHOLD) {
    // Obstacle detected in front, check sides
    if (left > right) {
      // More space on the left, turn left
      moveRobot(30, -30);
    } else if (right > left) {
      // More space on the right, turn right
      moveRobot(-40, 40); 
    } else {
      // Equal space on both sides or both sides blocked, turn around
      moveRobot(50, -50);
    }
    delay(300);
  } 
  else if ((left <= OBSTACLE_THRESHOLD && (right < 5) && right > OBSTACLE_THRESHOLD ) || (left <= CLOSE_OBSTACLE_THRESHOLD) ) {
    // Obstacle on the left, veer right
    moveRobot(20, 50);
  } else if ((right <= OBSTACLE_THRESHOLD && (left < 5 ) && left > OBSTACLE_THRESHOLD) || (right <= CLOSE_OBSTACLE_THRESHOLD)) {
    // Obstacle on the right, veer left
    moveRobot(50, 20);
  } else {
    // No obstacles, move forward
    moveRobot(50, 50);
  }
}

int irDetect(int irLedPin, int irReceiverPin, long frequency) {
  tone(irLedPin, frequency);
  delay(1);
  noTone(irLedPin);
  int ir = digitalRead(irReceiverPin);
  delay(1);
  return ir;
}

int irDistance(int irLedPin, int irReceiverPin) {
  int distance = 0;
  for (long f = 38000; f <= 42000; f += 1000) {
    distance += irDetect(irLedPin, irReceiverPin, f);
  }
  return distance;
}

void updateIndicatorLed(int ledPin, int distance) {
  digitalWrite(ledPin, distance < OBSTACLE_THRESHOLD ? HIGH : LOW);
}

void moveRobot(int leftSpeed, int rightSpeed) {
  int leftMs = map(leftSpeed, -100, 100, SERVO_MAX_REVERSE, SERVO_MAX_FORWARD);
  int rightMs = map(rightSpeed, -100, 100, SERVO_MAX_FORWARD, SERVO_MAX_REVERSE);

  leftServo.writeMicroseconds(leftMs);
  rightServo.writeMicroseconds(rightMs);
}

// Servo movement functions
void moveForward(int leftSpeed, int rightSpeed) {
  leftServo.writeMicroseconds(leftSpeed);   // Adjust left servo speed
  rightServo.writeMicroseconds(rightSpeed);  // Adjust right servo speed
}

void turnLeft() {
  leftServo.writeMicroseconds(1400);    // Stop left wheel
  rightServo.writeMicroseconds(1400);   // Move right wheel forward for turn
  delay(600);                        // Adjust this delay for 90-degree turn

}

void turnRight() {
  leftServo.writeMicroseconds( 1600);    // Move left wheel forward for turn
  rightServo.writeMicroseconds( 1600);   // Stop right wheel
  delay(600);                        // Adjust this delay for 90-degree turn

}

void stopMovement() {
  leftServo.writeMicroseconds(1495);    // Stop left wheel
  rightServo.writeMicroseconds(1490);   // Stop right wheel
}
