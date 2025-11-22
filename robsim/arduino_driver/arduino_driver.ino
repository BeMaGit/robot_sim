#include <Servo.h>

// Configuration
const int NUM_SERVOS = 7;
const int SERVO_PINS[NUM_SERVOS] = {3, 5, 6, 9, 10, 11, 13}; // Example pins, adjust as needed
// 0: Waist, 1: Shoulder, 2: Elbow, 3: Wrist Pitch, 4: Wrist Roll, 5: Gripper, 6: Aux/Extra

Servo servos[NUM_SERVOS];
float targetAngles[NUM_SERVOS];
String inputString = "";
boolean stringComplete = false;
unsigned long lastCommandTime = 0;
const unsigned long TIMEOUT_MS = 2000; // Stop if no command for 2 seconds

void setup() {
  Serial.begin(115200);
  inputString.reserve(200);

  // Attach servos
  for (int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(SERVO_PINS[i]);
    targetAngles[i] = 90.0; // Default center
    servos[i].write(targetAngles[i]);
  }
  
  Serial.println("READY");
}

void loop() {
  // Safety Timeout
  if (millis() - lastCommandTime > TIMEOUT_MS) {
    // Optional: Move to safe position or detach
    // for (int i = 0; i < NUM_SERVOS; i++) { servos[i].detach(); }
  }

  if (stringComplete) {
    parseCommand(inputString);
    // clear the string:
    inputString = "";
    stringComplete = false;
    lastCommandTime = millis();
  }
}

void parseCommand(String cmd) {
  // Expected format: "CMD:ang0,ang1,ang2,ang3,ang4,ang5,ang6"
  // Example: "CMD:90.0,45.5,120.0,..."
  
  if (cmd.startsWith("CMD:")) {
    String values = cmd.substring(4);
    int idx = 0;
    int lastIndex = 0;
    
    for (int i = 0; i < NUM_SERVOS; i++) {
      int nextIndex = values.indexOf(',', lastIndex);
      if (nextIndex == -1 && i < NUM_SERVOS - 1) {
        // Error: not enough values
        break;
      }
      if (i == NUM_SERVOS - 1) {
        nextIndex = values.length();
      }
      
      String valStr = values.substring(lastIndex, nextIndex);
      float angle = valStr.toFloat();
      
      // Constrain and write
      angle = constrain(angle, 0.0, 180.0);
      servos[i].write(angle);
      
      lastIndex = nextIndex + 1;
    }
    // Serial.println("ACK"); // Optional acknowledgment
  }
}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
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
