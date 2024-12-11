// codigo arduino sensor
#define linhaDir 2
#define linhaEsq 3


// Ultrasonic sensor pins
const int trigPin = 10;
const int echoPin = 11;

int esqDetect, dirDetect = 0;

// Robot states
enum RobotState { ACQUIRING_TARGET,
                  HOMING,
                  LOST_TRACK };

// Detection parameters
const int WINDOW_SIZE = 15;
const int MAX_DISTANCE = 100;        // cm
const int MIN_DISTANCE = 5;          // cm
const int HOMING_DISTANCE = 20;      // Distance to start aggressive homing
const int LOST_TRACK_THRESHOLD = 3;  // Consecutive measurements without target
const int MAX_SEARCH_SWEEPS = 3;

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
  int searchDirection = 1;   // 1 for right, -1 for left
  int searchSweepCount = 0;  // Track number of complete search sweeps
};

RobotControl robotControl;

void setup() {

  pinMode(linhaDir, INPUT_PULLUP);
  pinMode(linhaEsq, INPUT_PULLUP);
  // Ultrasonic sensor pin setup
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Initialize serial communication
  Serial.begin(9600);

  attachInterrupt(linhaDir, detectDir, FALLING);
  attachInterrupt(linhaEsq, detectEsq, FALLING);


  // Initialize window with zeros
  for (int i = 0; i < WINDOW_SIZE; i++) {
    robotControl.distanceWindow.measurements[i] = 0;
  }
  delay(5000);  //some daqui meu
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
  robotControl.distanceWindow
    .measurements[robotControl.distanceWindow.currentIndex] = distance;
  robotControl.distanceWindow.currentIndex =
    (robotControl.distanceWindow.currentIndex + 1) % WINDOW_SIZE;
}

// Calculate median distance from window
int calculateMedianDistance() {
  int sortedMeasurements[WINDOW_SIZE];

  // Copy measurements
  for (int i = 0; i < WINDOW_SIZE; i++) {
    sortedMeasurements[i] =
      robotControl.distanceWindow.measurements[i];
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
Serial.println("para");
  Serial.write("para"); //escreve pro outro arduino
}

void rotateClockwise(int speed = ROTATE_SPEED) {
Serial.println("2"); //escreve pro outro arduino

}

void rotateCounterClockwise(int speed = ROTATE_SPEED) {
Serial.println("3"); //escreve pro outro arduino

}

void moveForward(int speed = HOMING_SPEED) {
Serial.println("1");
}

// State-specific behaviors
void acquireTarget() {
  // Continuously rotate to find target
 
  rotateClockwise();

  // Check for potential target
  int medianDistance = calculateMedianDistance();
 // Serial.print("distance: ");
//Serial.println(medianDistance);


  if (medianDistance > MIN_DISTANCE && medianDistance < MAX_DISTANCE) {
    // Potential target found
    robotControl.currentState = HOMING;
    robotControl.targetDistance = medianDistance;

    //Serial.println("Target Acquired. Switching to Homing.");
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

     // Serial.println("Lost Track of Target. Searching.");
      return;
    }
  }
  if (currentDistance < MIN_DISTANCE) {
    //Serial.println("Empurra baixo nengue");
  } else {
    robotControl.lostTrackCounter = 0;
  }

  // Aggressive homing logic
  //if (currentDistance < HOMING_DISTANCE || dirDetect == 1) {
  if (currentDistance < HOMING_DISTANCE) {
    // Target is far, move forward aggressively
    moveForward();
    //Serial.println("O frank vai pega oce");
  } else {
    // Close to target, fine-tune positioning
    if (currentDistance > robotControl.targetDistance) {
      rotateClockwise(30);  // Slight adjustment
      //Serial.println("Homing - Fine Tuning");
    } else {
      rotateCounterClockwise(30);  // Slight adjustment
      //Serial.println("Homing - Fine Tuning");
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
    robotControl.searchDirection *=
    -1;                               // Alternate search direction next time
    robotControl.searchSweepCount = 0;  // Reset sweep count

    //Serial.println("Target Reacquired. Returning to Homing.");
    return;
  }

  // Complete sweep check
  if (abs(robotControl.searchDirection) == abs(1)) {
    robotControl.searchSweepCount++;

    // If maximum sweeps reached, return to acquiring target
    if (robotControl.searchSweepCount >= MAX_SEARCH_SWEEPS) {
      robotControl.currentState = ACQUIRING_TARGET;
      robotControl.searchSweepCount = 0;

      //Serial.println("Target not found after maximum sweeps. "
                    // "Returning to Acquisition.");
    }
  }
}

void detectEsq() {
  esqDetect = 1;
}

void detectDir() {
  dirDetect = 1;
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
