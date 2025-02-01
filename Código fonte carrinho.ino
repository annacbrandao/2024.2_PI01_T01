// Tanque lutador de sumô 

//conexões sensores de linha 
#define linhaDir 2
#define linhaEsq 3

//pinos para a ponte h
#define mot1a 5 //pwm
#define mot1b 6 //pwm
#define mot2a 9 //pwm
#define mot2b 10 //pwm

// conexões do sensor de distância
const int trigPin = 11;
const int echoPin = 12;

int esqDetect, dirDetect = 0;

//Estados do robo
enum RobotState { 
ACQUIRING_TARGET,
HOMING,
LOST_TRACK 
};

// Parâmetros de detecção em centímetros
const int JANELA = 15;
const int DISTANCIA_MAX = 1500;        
const int DISTANCIA_MIN = 5;          // distância de detecção do obstáculo 
const int HOMING_DISTANCE = 60;      // Distance to start aggressive homing
const int LOST_TRACK_THRESHOLD = 3;  // Consecutive measurements without target
const int MAX_SEARCH_SWEEPS = 3;

// Parâmetros de velocidade dos motores 
const int veloProcura = 120;  //meia potência 
const int HOMING_SPEED = 120; //meia potência
const int veloVolta = 127;    //potência normal
const int veloEmpurra = 255;  //forca máxima 

// Sliding window and state management
struct WindowData {
  int measurements[JANELA];
  int currentIndex = 0;
};

struct RobotControl {
  RobotState currentState = ACQUIRING_TARGET;
  WindowData distanceWindow;
  int lostTrackCounter = 0;
  int targetDistance = 0;
  int searchDirection = 1;   // 1 pra direita, -1 pra esquerda
  int searchSweepCount = 0;  // Track number of complete search sweeps
};

RobotControl robotControl;





void setup() {

  //pinos sensor linha
  pinMode(linhaDir, INPUT_PULLUP);
  pinMode(linhaEsq, INPUT_PULLUP);
  // Ultrasonic sensor pin setup
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

//MOTORES
  pinMode(mot1a, OUTPUT);
 pinMode(mot1b, OUTPUT);
 pinMode(mot2a, OUTPUT);
 pinMode(mot2b, OUTPUT);

  //Iniciar serial
  Serial.begin(115200);

//função para mudar a função quando acha a linha 
  attachInterrupt(digitalPinToInterrupt(linhaDir), detectDir, FALLING);
  attachInterrupt(digitalPinToInterrupt(linhaEsq), detectEsq, FALLING);


  // Inicializar a jenale com zeros
  for (int i = 0; i < JANELA; i++) {
    robotControl.distanceWindow.measurements[i] = 0;
  }

//tempo pra por o robô no chão 
  delay(4000);  
}




// Função pra medir a distância 
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
Serial.print(distance);
  return distance;
}

// Add measurement to sliding window
void addMeasurement(int distance) {
  robotControl.distanceWindow
    .measurements[robotControl.distanceWindow.currentIndex] = distance;
  robotControl.distanceWindow.currentIndex =
    (robotControl.distanceWindow.currentIndex + 1) % JANELA;
}

// Calcular a média das distâncias
int calculateMedianDistance() {
  int sortedMeasurements[JANELA];

  // Copy measurements
  for (int i = 0; i < JANELA; i++) {
    sortedMeasurements[i] =
      robotControl.distanceWindow.measurements[i];
  }

  // Insertion sort
  for (int i = 1; i < JANELA; i++) {
    int key = sortedMeasurements[i];
    int j = i - 1;

    while (j >= 0 && sortedMeasurements[j] > key) {
      sortedMeasurements[j + 1] = sortedMeasurements[j];
      j = j - 1;
    }
    sortedMeasurements[j + 1] = key;
  }

  // Return median
  return sortedMeasurements[JANELA / 2];
}


// Motor control functions
void moveForward() {
   analogWrite(mot1a, LOW);
  analogWrite(mot1b, 127);
 analogWrite(mot2a, LOW);
 analogWrite(mot2b, 127);
  Serial.println("pra frente");  //chama a funcao pra andar pra frente no arduino dos motores
  delay(20);
}

void empurra() {
 analogWrite(mot1a, LOW);
 analogWrite(mot1b, 170);
 analogWrite(mot2a, LOW);
 analogWrite(mot2b, 170);
  Serial.println("sai do meio");  //chama a funcao pra girar a terra ao contrário 
  delay(20);
}

void rotateClockwise() {
 analogWrite(mot1a, 120);
  digitalWrite(mot1b, LOW);
 digitalWrite(mot2a, LOW);
 analogWrite(mot2b, 120);
  Serial.println("peao da casa propia");  //chama a funcao pra andar pra frente no arduino dos motores
  delay(20);
}

void rotateCounterClockwise() {
 digitalWrite(mot1a, LOW);
 analogWrite(mot1b, 120);
 analogWrite(mot2a, 120);
 digitalWrite(mot2b, LOW);
  Serial.println("peao inverso");  //chama a funcao pra andar pra frente no arduino dos motores

  delay(20);
}

void volta() {

   digitalWrite(mot1a, LOW);
  digitalWrite(mot1b, veloVolta);
 digitalWrite(mot2a, LOW);
 digitalWrite(mot2b, veloVolta);
  Serial.println("volta");
delay(1000);

}

// State-specific behaviors
void acquireTarget() {
  // Continuously rotate to find target
  rotateClockwise();
//Serial.println("to aqui");
  // Check for potential target
  int medianDistance = calculateMedianDistance();

  if (medianDistance > DISTANCIA_MIN && medianDistance < DISTANCIA_MAX) {
    // Potential target found
    robotControl.currentState = HOMING;
    robotControl.targetDistance = medianDistance;

    Serial.println("Target Acquired. Switching to Homing.");
  }
}

void homeToTarget() {
  int currentDistance = calculateMedianDistance();

  // Check if target is lost
  if (currentDistance > DISTANCIA_MAX ) {
    robotControl.lostTrackCounter++;

    if (robotControl.lostTrackCounter >= LOST_TRACK_THRESHOLD) {
      robotControl.currentState = LOST_TRACK;
      robotControl.lostTrackCounter = 0;

      Serial.println("Lost Track of Target. Searching.");
      return;
    }
  }
  if (currentDistance <= DISTANCIA_MIN) {
empurra();
// sobe pra 200 de pwm pra ter mais força
    Serial.println("Empurra baixo nengue");
  } else {
    robotControl.lostTrackCounter = 0;
  }

  // Aggressive homing logic
  if (currentDistance < HOMING_DISTANCE && currentDistance > DISTANCIA_MIN) {
    // Target is far, move forward aggressively
    moveForward();
    Serial.println("O frank vai pega oce");
  }
    /* else {
     //Close to target, fine-tune positioning
    if (currentDistance > robotControl.targetDistance) {
      rotateClockwise();  // Slight adjustment
   Serial.println("Homing - Fine Tuning");
    } else {
    rotateCounterClockwise();  // Slight adjustment
      Serial.println("Homing - Fine Tuning");
    }
  
    }*/
}

void lostTrackSearch() {
  // Alternate search direction
  if (robotControl.searchDirection > 0) {
    // Search right
    rotateClockwise();
  } else {
    // Search left
    rotateCounterClockwise();
  }

  int currentDistance = calculateMedianDistance();

  // Check if target is reacquired
  if (currentDistance > DISTANCIA_MIN && currentDistance < DISTANCIA_MAX) {
    robotControl.currentState = HOMING;
    robotControl.targetDistance = currentDistance;
    robotControl.searchDirection *=
      -1;                               // Alternate search direction next time
    robotControl.searchSweepCount = 0;  // Reset sweep count

    Serial.println("Target Reacquired. Returning to Homing.");
    return;
  }

  // Complete sweep check
  if (abs(robotControl.searchDirection) == abs(1)) {
    robotControl.searchSweepCount++;

    // If maximum sweeps reached, return to acquiring target
    if (robotControl.searchSweepCount >= MAX_SEARCH_SWEEPS) {
      robotControl.currentState = ACQUIRING_TARGET;
      robotControl.searchSweepCount = 0;

      Serial.println("Target not found after maximum sweeps. "
                     "Returning to Acquisition.");
    }
  }
}

void detectEsq() {
  esqDetect = 1;
volta();
}

void detectDir() {
  dirDetect = 1;
volta();
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
