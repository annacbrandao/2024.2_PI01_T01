// Motor control pins
const int motorLeftForward = 47;
const int motorLeftReverse = 45;
const int motorRightForward = 49;
const int motorRightReverse = 44;

// Ultrasonic sensor pins
const int trigPin = 25;
const int echoPin = 27;

// Robot states
enum RobotState {
  ACQUIRING_TARGET,
  HOMING,
  LOST_TRACK
};

// Detection parameters
const int WINDOW_SIZE = 15;
const int MAX_DISTANCE = 100;  // cm
const int MIN_DISTANCE = 5;    // cm
const int HOMING_DISTANCE = 20;  // Distance to start aggressive homing
const int LOST_TRACK_THRESHOLD = 3;  // Consecutive measurements without target

// Motor and movement parameters
const int ROTATE_SPEED = 50;
const int HOMING_SPEED = 100;
const int SEARCH_ANGLE_LEFT = -45;  // Degrees to look left when lost
const int SEARCH_ANGLE_RIGHT = 45;  // Degrees to look right when lost

// Sliding window and state management
struct WindowData {
  int measurements[WINDOW_SIZE];
  int currentIndex = 0;
};

struct RobotControl {
  RobotState currentState = ACQUIRING_TARGET;
  WindowData distanceWindow;
  int lostTrackCounter = 0;
  int targetDistance = 0;
  int searchDirection = 1;  // 1 for right, -1 for left
};

RobotControl robotControl;

void setup() {
  // Motor pin setup
  pinMode(motorLeftForward, OUTPUT);
  pinMode(motorLeftReverse, OUTPUT);
  pinMode(motorRightForward, OUTPUT);
  pinMode(motorRightReverse, OUTPUT);

  // Ultrasonic sensor pin setup
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Initialize serial communication
  Serial.begin(9600);

  // Initialize window with zeros
  for (int i = 0; i < WINDOW_SIZE; i++) {
    robotControl.distanceWindow.measurements[i] = 0;
  }
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

// Add measurement to sliding window
void addMeasurement(int distance) {
  robotControl.distanceWindow.measurements[robotControl.distanceWindow.currentIndex] = distance;
  robotControl.distanceWindow.currentIndex = (robotControl.distanceWindow.currentIndex + 1) % WINDOW_SIZE;
}

// Calculate median distance from window
int calculateMedianDistance() {
  int sortedMeasurements[WINDOW_SIZE];
  
  // Copy measurements
  for (int i = 0; i < WINDOW_SIZE; i++) {
    sortedMeasurements[i] = robotControl.distanceWindow.measurements[i];
  }
  
  // Insertion sort
  for (int i = 1; i < WINDOW_SIZE; i++) {
    int key = sortedMeasurements[i];
    int j = i - 1;

    while (j >= 0 && sortedMeasurements[j] > key) {
      sortedMeasurements[j + 1] = sortedMeasurements[j];
      j = j - 1;
    }
    sortedMeasurements[j + 1] = key;
  }
  
  // Return median
  return sortedMeasurements[WINDOW_SIZE / 2];
}

// Motor control functions
void stopMotors() {
  digitalWrite(motorLeftForward, LOW);
  digitalWrite(motorLeftReverse, LOW);
  digitalWrite(motorRightForward, LOW);
  digitalWrite(motorRightReverse, LOW);
}

void rotateClockwise(int speed = ROTATE_SPEED) {
  digitalWrite(motorLeftForward, HIGH);
  digitalWrite(motorLeftReverse, LOW);
  digitalWrite(motorRightForward, LOW);
  digitalWrite(motorRightReverse, HIGH);
  delay(50);
}

void rotateCounterClockwise(int speed = ROTATE_SPEED) {
  digitalWrite(motorLeftForward, LOW);
  digitalWrite(motorLeftReverse, HIGH);
  digitalWrite(motorRightForward, HIGH);
  digitalWrite(motorRightReverse, LOW);
  delay(50);
}

void moveForward(int speed = HOMING_SPEED) {
  digitalWrite(motorLeftForward, HIGH);
  digitalWrite(motorLeftReverse, LOW);
  digitalWrite(motorRightForward, HIGH);
  digitalWrite(motorRightReverse, LOW);
}

// State-specific behaviors
void acquireTarget() {
  // Continuously rotate to find target
  rotateClockwise();
  
  // Check for potential target
  int medianDistance = calculateMedianDistance();
  
  if (medianDistance > MIN_DISTANCE && medianDistance < MAX_DISTANCE) {
    // Potential target found
    robotControl.currentState = HOMING;
    robotControl.targetDistance = medianDistance;
    
    Serial.println("Target Acquired. Switching to Homing.");
  }
}

void homeToTarget() {
  int currentDistance = calculateMedianDistance();
  
  // Check if target is lost
  if (currentDistance > MAX_DISTANCE || currentDistance < MIN_DISTANCE) {
    robotControl.lostTrackCounter++;
    
    if (robotControl.lostTrackCounter >= LOST_TRACK_THRESHOLD) {
      robotControl.currentState = LOST_TRACK;
      robotControl.lostTrackCounter = 0;
      
      Serial.println("Lost Track of Target. Searching.");
      return;
    }
  } else {
    robotControl.lostTrackCounter = 0;
  }
  
  // Aggressive homing logic
  if (currentDistance > HOMING_DISTANCE) {
    // Target is far, move forward aggressively
    moveForward();
    Serial.println("Homing - Moving Forward");
  } else {
    // Close to target, fine-tune positioning
    if (currentDistance < robotControl.targetDistance) {
      rotateClockwise(30);  // Slight adjustment
      Serial.println("Homing - Fine Tuning");
    } else {
      rotateCounterClockwise(30);  // Slight adjustment
      Serial.println("Homing - Fine Tuning");
    }
  }
}

void lostTrackSearch() {
  // Alternate search direction
  if (robotControl.searchDirection > 0) {
    // Search right
    rotateClockwise(ROTATE_SPEED);
  } else {
    // Search left
    rotateCounterClockwise(ROTATE_SPEED);
  }
  
  int currentDistance = calculateMedianDistance();
  
  // Check if target is reacquired
  if (currentDistance > MIN_DISTANCE && currentDistance < MAX_DISTANCE) {
    robotControl.currentState = HOMING;
    robotControl.targetDistance = currentDistance;
    robotControl.searchDirection *= -1;  // Alternate search direction next time
    
    Serial.println("Target Reacquired. Returning to Homing.");
    return;
  }
}

void loop() {
  // Measure and add to sliding window
  int distance = measureDistance();
  addMeasurement(distance);
  
  // State machine
  switch (robotControl.currentState) {
    case ACQUIRING_TARGET:
      acquireTarget();
      break;
    
    case HOMING:
      homeToTarget();
      break;
    
    case LOST_TRACK:
      lostTrackSearch();
      break;
  }
  
  delay(50);  // Control processing rate
}
