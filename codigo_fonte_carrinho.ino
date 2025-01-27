// Tanque lutador de sumô 

//conexões sensores de linha 
#define linhaDir 2
#define linhaEsq 3

//pinos para a ponte h
#define mot1a 5 //pwm
#define mot1b 6 //pwm
#define mot2a 9 //pwm
#define mot2b 10 //pwm

#define ledPin 13


// conexões do sensor de distância
const int trigPin = 11;
const int echoPin = 12;

int esqDetect, dirDetect = 0;

//Estados do robo
enum RobotState { 
ACQUIRING_TARGET,
HOMING,
LOST_TRACK,
LINHAd_ENCONTRADA,
LINHAe_ENCONTRADA 
};

// Parâmetros de detecção em centímetros
const int JANELA = 15;
const int DISTANCIA_MAX = 1500;        
const int DISTANCIA_MIN = 5;          // distância de detecção do obstáculo 
const int HOMING_DISTANCE = 60;      // distancia to start aggressive homing
const int LOST_TRACK_THRESHOLD = 3;  // quantas medicoes sem achar o alvo?
const int MAX_SEARCH_SWEEPS = 3;

// Parâmetros de velocidade dos motores 
const int veloProcura = 120;  //meia potência 
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
  int targetdistance = 0;
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
    
    pinMode(ledPin, OUTPUT);

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
  delay(5000);  
}




// Função pra medir a distância 
int medirDistancia() {
 
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // medir duracao of pulse from echo pin
  long duracao = pulseIn(echoPin, HIGH);

  // Calcula a distancia em centimetros
  int distancia = duracao * 0.034 / 2;
Serial.print(distancia);
  return distancia;
}

//adiciona medicoes a janela deslizante
void addmeasurement(int distancia) {
  robotControl.distanceWindow
    .measurements[robotControl.distanceWindow.currentIndex] = distancia;
  robotControl.distanceWindow.currentIndex =
    (robotControl.distanceWindow.currentIndex + 1) % JANELA;
}

// Calcular a média das distâncias
int calculateMediandistance() {
  int sortedmeasurements[JANELA];

  //copia as medicoes
  for (int i = 0; i < JANELA; i++) {
    sortedmeasurements[i] =
      robotControl.distanceWindow.measurements[i];
  }

  // Insertion sort
  for (int i = 1; i < JANELA; i++) {
    int key = sortedmeasurements[i];
    int j = i - 1;

    while (j >= 0 && sortedmeasurements[j] > key) {
      sortedmeasurements[j + 1] = sortedmeasurements[j];
      j = j - 1;
    }
    sortedmeasurements[j + 1] = key;
  }

  // Return median
  return sortedmeasurements[JANELA / 2];
}


// Motor control functions
void praFrente() {
   analogWrite(mot1a, veloProcura);
  digitalWrite(mot1b, LOW);
 analogWrite(mot2a, veloProcura);
 digitalWrite(mot2b, LOW);
  Serial.println("pra frente");  //chama a funcao pra andar pra frente no arduino dos motores
  delay(20);
}

void empurra() {
   analogWrite(mot1a, veloEmpurra);
  digitalWrite(mot1b, LOW);
 analogWrite(mot2a, veloEmpurra);
 digitalWrite(mot2b, LOW);
  Serial.println("sai do meio");  //chama a funcao pra girar a terra ao contrário 
  delay(20);
}

void rotateClockwise() {
 analogWrite(mot1a, veloProcura);
  digitalWrite(mot1b, LOW);
 digitalWrite(mot2a, LOW);
 digitalWrite(mot2b, veloProcura);
  Serial.println("peao da casa propia");  //chama a funcao pra andar pra frente no arduino dos motores
  delay(20);
}

void rotateCounterClockwise() {
 digitalWrite(mot1a, LOW);
 digitalWrite(mot1b, veloProcura);
 analogWrite(mot2a, veloProcura);
 digitalWrite(mot2b, LOW);
  Serial.println("peao inverso");  //chama a funcao pra andar pra frente no arduino dos motores

  delay(20);
}

void detectEsq() {
if(digitalRead(linhaEsq)==LOW){

    digitalWrite(mot1a, LOW);
    digitalWrite(mot1b, veloVolta);
    digitalWrite(mot2a, LOW);
    digitalWrite(mot2b, veloVolta);
      Serial.println("volta esquerda");
    
    digitalWrite(ledPin,HIGH);
    delay(1000);
    digitalWrite(ledPin,LOW);
  }
}

void detectDir() {
if(digitalRead(linhaDir)==LOW){
digitalWrite(mot1a, LOW);
    digitalWrite(mot1b, veloVolta);
    digitalWrite(mot2a, LOW);
    digitalWrite(mot2b, veloVolta);
      Serial.println("volta direita");
    digitalWrite(ledPin,HIGH);
    delay(1000);
    digitalWrite(ledPin,LOW);
  }
}


// State-specific behaviors
void acquireTarget() {
  // Continuously rotate to find target
  rotateClockwise();

  // Check for potential target
  int medianDistance = calculateMediandistance();

  if (medianDistance > DISTANCIA_MIN && medianDistance < DISTANCIA_MAX) {
    // Potential target found
    robotControl.currentState = HOMING;
    robotControl.targetdistance = medianDistance;

    Serial.println("achei");
  }
}

void homeToTarget() {
  int currentdistance = calculateMediandistance();

  // Check if target is lost
  if (currentdistance > DISTANCIA_MAX || currentdistance < DISTANCIA_MIN) {
    robotControl.lostTrackCounter++;

    if (robotControl.lostTrackCounter >= LOST_TRACK_THRESHOLD) {
      robotControl.currentState = LOST_TRACK;
      robotControl.lostTrackCounter = 0;

      Serial.println("me perdi.");
      return;
    }
  }
  if (currentdistance < DISTANCIA_MIN) {
empurra();
    Serial.println("SAI DO MEI");
    if(digitalRead(linhaDir)==LOW){
      robotControl.currentState = LINHAd_ENCONTRADA;
    }
    if(digitalRead(linhaEsq)==LOW){
      robotControl.currentState = LINHAe_ENCONTRADA;
    }
  } else {
    robotControl.lostTrackCounter = 0;
  }

  // Aggressive homing logic
  if (currentdistance < HOMING_DISTANCE) {
    // Target is far, move forward aggressively
    praFrente();
    Serial.println("O frank vai pega oce");
  } else {
    // Close to target, fine-tune positioning
    if (currentdistance > robotControl.targetdistance) {
      rotateClockwise();  // Slight adjustment
      Serial.println("to me achando");
    } else {
      rotateCounterClockwise();  // Slight adjustment
      Serial.println("to me achando");
    }
  }
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

  int currentdistance = calculateMediandistance();

  // Check if target is reacquired
  if (currentdistance > DISTANCIA_MIN && currentdistance < DISTANCIA_MAX) {
    robotControl.currentState = HOMING;
    robotControl.targetdistance = currentdistance;
    robotControl.searchDirection *=
      -1;                               // Alternate search direction next time
    robotControl.searchSweepCount = 0;  // Reset sweep count

    Serial.println("achei");
    return;
  }

  // Complete sweep check
  if (abs(robotControl.searchDirection) == abs(1)) {
    robotControl.searchSweepCount++;

    // If maximum sweeps reached, return to acquiring target
    if (robotControl.searchSweepCount >= MAX_SEARCH_SWEEPS) {
      robotControl.currentState = ACQUIRING_TARGET;
      robotControl.searchSweepCount = 0;

      Serial.println("to mais perdido q cego em tiroteio");
    }
  }
}


void loop() {
  // medir and add to sliding window
  int distancia = medirDistancia();
  addmeasurement(distancia);

  // maquina de estados
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

    case LINHAd_ENCONTRADA:
       detectDir();
      break;
    case LINHAe_ENCONTRADA:
       detectEsq();
      break;      
  }
  delay(50);  // Control processing rate
}
