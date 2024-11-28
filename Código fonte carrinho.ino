#include <Servo.h>

// Motor control pins
const int motorLeftForward = 47;
const int motorLeftReverse = 45;
const int motorRightForward = 49;
const int motorRightReverse = 44;

// Ultrasonic sensor pins
const int trigPin = 25;
const int echoPin = 27;

// Sliding window parameters
const int WINDOW_SIZE = 10;  // Increased window size for more stable readings
const int DETECTION_THRESHOLD = 7;  // Require more consecutive detections
const int MAX_DISTANCE = 50;  // Maximum distance to consider an object detected (cm)
const int MIN_DISTANCE = 10;  // Minimum distance to avoid false positives (cm)

// Centering parameters
const int CENTERING_TOLERANCE = 3;  // Acceptable range for centering (cm)
const int MAX_CENTERING_ATTEMPTS = 5;  // Limit centering attempts to prevent infinite loops

// Sliding window for distance measurements
int distanceWindow[WINDOW_SIZE] = {0};
int windowIndex = 0;

// Tracking object detection and centering
struct ObjectDetection {
  bool detected = false;
  int lastDistance = 0;
  int centeringAttempts = 0;
} objectDetection;

void setup() {
  // Motor pin setup
  pinMode(motorLeftForward, OUTPUT);
  pinMode(motorLeftReverse, OUTPUT);
  pinMode(motorRightForward, OUTPUT);
  pinMode(motorRightReverse, OUTPUT);

  // Ultrasonic sensor pin setup
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Initialize serial communication for debugging
  Serial.begin(9600);
}

// Function to measure distance using ultrasonic sensor
int measureDistance() {
  // Clear the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // Send a 10 microsecond pulse
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Measure duration of pulse from echo pin
  long duration = pulseIn(echoPin, HIGH);
  
  // Calculate distance in centimeters
  int distance = duration * 0.034 / 2;
  
  return distance;
}

// Function to rotate the car clockwise
void rotateClockwise(int duration = 50) {
  // Left motor forward
  digitalWrite(motorLeftForward, HIGH);
  digitalWrite(motorLeftReverse, LOW);
  
  // Right motor reverse
  digitalWrite(motorRightForward, LOW);
  digitalWrite(motorRightReverse, HIGH);
  
  delay(duration);
}

// Function to rotate the car counterclockwise
void rotateCounterClockwise(int duration = 50) {
  // Left motor reverse
  digitalWrite(motorLeftForward, LOW);
  digitalWrite(motorLeftReverse, HIGH);
  
  // Right motor forward
  digitalWrite(motorRightForward, HIGH);
  digitalWrite(motorRightReverse, LOW);
  
  delay(duration);
}

// Function to stop the car
void stopMotors() {
  digitalWrite(motorLeftForward, LOW);
  digitalWrite(motorLeftReverse, HIGH);
  digitalWrite(motorRightForward, LOW);
  digitalWrite(motorRightReverse, HIGH);
}

// Update sliding window with new distance measurement
void updateDistanceWindow(int newDistance) {
  distanceWindow[windowIndex] = newDistance;
  windowIndex = (windowIndex + 1) % WINDOW_SIZE;
}

// Check if object is detected using sliding window
bool isObjectDetected() {
  int detectionCount = 0;
  
  for (int i = 0; i < WINDOW_SIZE; i++) {
    if (distanceWindow[i] > MIN_DISTANCE && distanceWindow[i] < MAX_DISTANCE) {
      detectionCount++;
    }
  }
  
  return (detectionCount >= DETECTION_THRESHOLD);
}

// Find the median distance in the sliding window
int findMedianDistance() {
  // Create a copy of the window to sort
  int sortedWindow[WINDOW_SIZE];
  memcpy(sortedWindow, distanceWindow, sizeof(distanceWindow));
  
  // Simple bubble sort
  for (int i = 0; i < WINDOW_SIZE - 1; i++) {
    for (int j = 0; j < WINDOW_SIZE - i - 1; j++) {
      if (sortedWindow[j] > sortedWindow[j + 1]) {
        int temp = sortedWindow[j];
        sortedWindow[j] = sortedWindow[j + 1];
        sortedWindow[j + 1] = temp;
      }
    }
  }
  
  // Return median (middle value)
  return sortedWindow[WINDOW_SIZE / 2];
}

// Attempt to center the car on the detected object
void centerOnObject() {
  int currentDistance = findMedianDistance();
  
  // Check if we've exceeded max centering attempts
  if (objectDetection.centeringAttempts >= MAX_CENTERING_ATTEMPTS) {
    Serial.println("Max centering attempts reached");
    objectDetection.detected = false;
    objectDetection.centeringAttempts = 0;
    return;
  }
  
  // Compare current distance with last measured distance
  int distanceDiff = abs(currentDistance - objectDetection.lastDistance);
  
  Serial.print("Centering - Current Distance: ");
  Serial.print(currentDistance);
  Serial.print(", Last Distance: ");
  Serial.print(objectDetection.lastDistance);
  Serial.print(", Difference: ");
  Serial.println(distanceDiff);
  
  // If within acceptable tolerance, stop
  if (distanceDiff <= CENTERING_TOLERANCE) {
    stopMotors();
    Serial.println("pra frente!");
    return;
  }
  
  // Rotate based on distance change
  if (currentDistance < objectDetection.lastDistance) {
    // Object is closer, rotate slightly counterclockwise
    rotateCounterClockwise(75);
  } else {
    // Object is further, rotate slightly clockwise
    rotateClockwise(75);
  }
  
  // Update tracking
  objectDetection.lastDistance = currentDistance;
  objectDetection.centeringAttempts++;
}

void loop() {
  // Measure distance
  int distance = measureDistance();
  
  // Update sliding window
  updateDistanceWindow(distance);
  
  // Check if object is detected
  if (isObjectDetected()) {
    if (!objectDetection.detected) {
      // First detection, initialize tracking
      objectDetection.detected = true;
      objectDetection.lastDistance = findMedianDistance();
      objectDetection.centeringAttempts = 0;
      Serial.println("Object First Detected");
    }
    
    // Attempt to center on the object
    centerOnObject();
  } else {
    // Continue searching if no object detected
    if (objectDetection.detected) {
      // Reset detection if previously detected
      objectDetection.detected = false;
      objectDetection.centeringAttempts = 0;
    }
    
    // Rotate to search
    rotateClockwise();
  }
  
  // Small delay to control processing
  delay(50);
}
