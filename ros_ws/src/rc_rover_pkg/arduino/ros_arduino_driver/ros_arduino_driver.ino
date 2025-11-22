#include <Servo.h>

// Configuration
const int NUM_SERVOS = 6;
const int SERVO_PINS[NUM_SERVOS] = {3, 5, 6, 9, 10, 11}; 
// 0: Waist, 1: Shoulder, 2: Elbow, 3: Wrist Pitch, 4: Wrist Roll, 5: Gripper

// Motor Pins (L298N)
// Front Left
const int ENA = 2;
const int IN1 = 22;
const int IN2 = 23;
// Front Right
const int ENB = 4;
const int IN3 = 24;
const int IN4 = 25;
// Rear Left
const int ENC = 7;
const int IN5 = 26;
const int IN6 = 27;
// Rear Right
const int END = 8;
const int IN7 = 28;
const int IN8 = 29;

Servo servos[NUM_SERVOS];
String inputString = "";
boolean stringComplete = false;
unsigned long lastCommandTime = 0;
const unsigned long TIMEOUT_MS = 2000;

void setup() {
  Serial.begin(115200);
  inputString.reserve(200);

  // Attach servos
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(SERVO_PINS[i]);
    servos[i].write(90); // Default center
  }

  // Setup Motor Pins
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENC, OUTPUT); pinMode(IN5, OUTPUT); pinMode(IN6, OUTPUT);
  pinMode(END, OUTPUT); pinMode(IN7, OUTPUT); pinMode(IN8, OUTPUT);

  stopMotors();
  
  Serial.println("READY");
}

void loop() {
  // Safety Timeout
  if (millis() - lastCommandTime > TIMEOUT_MS) {
    stopMotors();
  }

  if (stringComplete) {
    parseCommand(inputString);
    inputString = "";
    stringComplete = false;
    lastCommandTime = millis();
  }
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); analogWrite(ENA, 0);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); analogWrite(ENB, 0);
  digitalWrite(IN5, LOW); digitalWrite(IN6, LOW); analogWrite(ENC, 0);
  digitalWrite(IN7, LOW); digitalWrite(IN8, LOW); analogWrite(END, 0);
}

void setMotor(int en, int in1, int in2, int speed) {
  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    speed = -speed;
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  analogWrite(en, constrain(speed, 0, 255));
}

void parseCommand(String cmd) {
  // Format: CMD:s1,s2,s3,s4,s5,s6,m1,m2,m3,m4
  if (cmd.startsWith("CMD:")) {
    String values = cmd.substring(4);
    int idx = 0;
    int lastIndex = 0;
    float params[10]; // 6 servos + 4 motors
    
    for (int i = 0; i < 10; i++) {
      int nextIndex = values.indexOf(',', lastIndex);
      if (nextIndex == -1 && i < 9) break; // Error
      if (i == 9) nextIndex = values.length();
      
      String valStr = values.substring(lastIndex, nextIndex);
      params[i] = valStr.toFloat();
      lastIndex = nextIndex + 1;
    }

    // Update Servos
    for (int i = 0; i < NUM_SERVOS; i++) {
      servos[i].write(constrain(params[i], 0, 180));
    }

    // Update Motors (Map -100 to 100 -> PWM)
    // M1: Front Left, M2: Front Right, M3: Rear Left, M4: Rear Right
    setMotor(ENA, IN1, IN2, map(params[6], -100, 100, -255, 255));
    setMotor(ENB, IN3, IN4, map(params[7], -100, 100, -255, 255));
    setMotor(ENC, IN5, IN6, map(params[8], -100, 100, -255, 255));
    setMotor(END, IN7, IN8, map(params[9], -100, 100, -255, 255));
  }
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}
