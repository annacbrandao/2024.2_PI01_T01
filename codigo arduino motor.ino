//codigo para mover os motores
#define M0 8
#define M1 9
#define M2 10
#define dirPin 4
#define stepPin 5  //pwm de pulsos pra passo
#define dirPin1 6
#define stepPin1 7     //pwm de pulsos pra passo
#define stepsVolta 200  //1.8 grau por passo




void setup() {
  // Motor pin setup
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);




  // Initialize serial communication
  Serial.begin(9600);
}

// Motor control functions
void stopMotors() {
  digitalWrite(dirPin, LOW);  //sentido de giro high ou low
  digitalWrite(dirPin1, LOW);  //sentido de giro high ou low
}

void rotateClockwise() {
  
  digitalWrite(dirPin, LOW);  //sentido de giro high ou low
      digitalWrite(dirPin1, HIGH);  //sentido de giro high ou low
  for (int i = 0; i < stepsVolta; i++) {
        
    digitalWrite(stepPin, HIGH);  //gira pra um lado
        digitalWrite(stepPin1, HIGH);  //gira pra um lado
    delayMicroseconds(1000);

    digitalWrite(stepPin, LOW);  //gira ao contrario
    digitalWrite(stepPin1, LOW);  //gira ao contrario
    delayMicroseconds(1000);
    //delay(500);
  
  }
}

void rotateCounterClockwise() {
  digitalWrite(dirPin, HIGH);  //sentido de giro high ou low
      digitalWrite(dirPin1, LOW);  //sentido de giro high ou low
  for (int i = 0; i < stepsVolta; i++) {
    
    digitalWrite(stepPin, HIGH);  //gira pra um lado
        digitalWrite(stepPin1, HIGH);  //gira pra um lado
    delayMicroseconds(1000);

    digitalWrite(stepPin, LOW);  //gira ao contrario
     digitalWrite(stepPin1, LOW);  //gira ao contrario
    delayMicroseconds(1000);
  

  }
}

void moveForward() {
  //Serial.println("a");
  digitalWrite(dirPin, HIGH);  //sentido de giro high ou low
      digitalWrite(dirPin1, HIGH);  //sentido de giro high ou low

  for (int i = 0; i < stepsVolta; i++) {
    digitalWrite(stepPin, HIGH);  //gira pra um lado
        digitalWrite(stepPin1, HIGH);  //gira pra um lado
    delayMicroseconds(1000);

    digitalWrite(stepPin, LOW);  //gira ao contrario
          digitalWrite(stepPin1, LOW);  //gira ao contrario
    delayMicroseconds(1000);
    
  
  }
}

void loop() {
 if (Serial.available() > 0) { // Verifica se há dados disponíveis na porta serial
    int received = Serial.parseInt(); // Lê o valor recebido e converte para inteiro
    if (received == 1) {
     moveForward();
    } else if (received == 2) {
      rotateClockwise();
    }else if (received == 3) {
      rotateCounterClockwise();
    }
  }
}
