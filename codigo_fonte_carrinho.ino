// Tanque lutador de sumô 
// Conexões sensores de linha
#define linhaDir 2
#define linhaEsq 3

// Pinos para a ponte H
#define mot1a 5 // PWM
#define mot1b 6
#define mot2a 9 // PWM
#define mot2b 10

// Conexões do sensor de distância
const int trigPin = 11;
const int echoPin = 12;

// Variáveis para detecção de linha
volatile bool lineDetectedFlag = false;

// Estados do robô
enum RobotState {
    IDLE,
    SEARCHING,
    APPROACHING_TARGET,
    RECOVERING_TARGET,
    AVOIDING_LINE
};

RobotState currentState = IDLE; // Estado inicial

// Parâmetros de detecção em centímetros
const int JANELA = 32;
const int DISTANCIA_MAX = 1500;
const int DISTANCIA_MIN = 8; // Distância de detecção do obstáculo
const int HOMING_DISTANCE = 60; // Distância para começar a perseguição agressiva
const int LOST_TRACK_THRESHOLD = 3; // Medições consecutivas sem detectar o alvo
const int MAX_SEARCH_SWEEPS = 3; // Número máximo de varreduras para recuperar o alvo

// Variáveis de controle
int lostTrackCount = 0;
int searchSweepCount = 0;
unsigned long lastDetectionTime = 0;

// Funções de movimento
void moveForward() {
    digitalWrite(mot1a, LOW);
    analogWrite(mot1b, 130);
    digitalWrite(mot2a, LOW);
    analogWrite(mot2b, 130);
}

void moveBackward() {
    analogWrite(mot1a, 130);
    digitalWrite(mot1b, LOW);
    analogWrite(mot2a, 130);
    digitalWrite(mot2b, LOW);
}

void rotateClockwise() {
    analogWrite(mot1a, 130);
    digitalWrite(mot1b, LOW);
    digitalWrite(mot2a, LOW);
    analogWrite(mot2b, 130);
}

void rotateCounterClockwise() {
    digitalWrite(mot1a, LOW);
    analogWrite(mot1b, 130);
    analogWrite(mot2a, 130);
    digitalWrite(mot2b, LOW);
}

void stopMotors() {
    digitalWrite(mot1a, LOW);
    digitalWrite(mot1b, LOW);
    digitalWrite(mot2a, LOW);
    digitalWrite(mot2b, LOW);
}

// Função para medir distância com o sensor ultrassônico
float measureDistance() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH);
    float distance = duration * 0.034 / 2; // Converter para centímetros
    Serial.println(distance);
    delay(10);
    return distance;
}

// Função para verificar se o alvo foi perdido
bool isTargetLost() {
    float distance = measureDistance();
    if (distance > HOMING_DISTANCE) {
        lostTrackCount++;
        if (lostTrackCount >= LOST_TRACK_THRESHOLD) {
            lostTrackCount = 0;
            return true;
        }
    } else {
        lostTrackCount = 0;
    }
    return false;
}

// Função para realizar pequenas varreduras à esquerda e à direita
void performSearchSweeps() {

        rotateCounterClockwise();
        delay(1000);
      Serial.print("procurando");
        if (measureDistance() <= HOMING_DISTANCE) {
          //stopMotors();
            return; // Alvo encontrado
        }
      

   // stopMotors();
}

void handleLineDetection() {
    static unsigned long lastInterruptTime = 0;
    unsigned long interruptTime = millis();
    if (interruptTime - lastInterruptTime > 200) { // Debouncing
        lineDetectedFlag = true;
    }
    lastInterruptTime = interruptTime;
}

void setup() {
    // Configuração dos pinos
    pinMode(linhaDir, INPUT);
    pinMode(linhaEsq, INPUT);
    pinMode(mot1a, OUTPUT);
    pinMode(mot1b, OUTPUT);
    pinMode(mot2a, OUTPUT);
    pinMode(mot2b, OUTPUT);
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);

    // Configuração das interrupções
    attachInterrupt(digitalPinToInterrupt(linhaDir), handleLineDetection, RISING);
    attachInterrupt(digitalPinToInterrupt(linhaEsq), handleLineDetection, RISING);

    // Inicialização
    Serial.begin(9600);
    stopMotors();
}

void loop() {
    if (lineDetectedFlag) {
        lineDetectedFlag = false;
        currentState = AVOIDING_LINE;
    }


// Máquina de estados
    switch (currentState) {
        case IDLE:
            // Aguarda um comando para começar
            currentState = SEARCHING;
            break;

        case SEARCHING:
            // Gira no sentido horário procurando um alvo
            rotateClockwise();
            if (measureDistance() <= HOMING_DISTANCE) {
                stopMotors();
                currentState = APPROACHING_TARGET;
            }
            break;

        case APPROACHING_TARGET:
            // Move-se em direção ao alvo
            moveForward();
            if (isTargetLost()) {
                stopMotors();
                currentState = RECOVERING_TARGET;
            }
            break;

        case RECOVERING_TARGET:
            // Realiza pequenas varreduras para tentar reencontrar o alvo
            //performSearchSweeps();
            Serial.print("kd???");
            if (measureDistance() <= HOMING_DISTANCE) {
                currentState = APPROACHING_TARGET;
            } else {
                searchSweepCount++;
                if (searchSweepCount >= MAX_SEARCH_SWEEPS) {
                    searchSweepCount = 0;
                    currentState = SEARCHING;
                }
            }
            break;

        case AVOIDING_LINE:
            // Recua e volta a procurar
            moveBackward();
            delay(2000);
            stopMotors();
            rotateClockwise();
            delay(2000);
            currentState = SEARCHING;
            break;
    }
}
