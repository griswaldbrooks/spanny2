#include <Servo.h>

Servo servo1;
Servo servo2;

int angle1 = 90; // Initial angle for servo 1
int angle2 = 90; // Initial angle for servo 2

void setup() {
  // shoulder
  servo1.attach(9); // Assuming servo 1 is attached to pin 9
  // elbow
  servo2.attach(10); // Assuming servo 2 is attached to pin 10
  
  servo1.write(angle1);
  servo2.write(angle2);
  
  Serial.begin(9600);
  Serial.println("Servo Control Started. Use h, j, k, l to control servos.");
}

void loop() {
  if (Serial.available()) {
    char ch = Serial.read();
    
    switch(ch) {
      case 'h':
        angle1 = max(0, angle1 - 5);
        servo1.write(angle1);
        break;
      case 'H':
        angle1 = max(0, angle1 - 1);
        servo1.write(angle1);
        break;
      case 'j':
        angle1 = min(180, angle1 + 5);
        servo1.write(angle1);
        break;
      case 'J':
        angle1 = min(180, angle1 + 1);
        servo1.write(angle1);
        break;
      case 'k':
        angle2 = max(0, angle2 - 5);
        servo2.write(angle2);
        break;
      case 'K':
        angle2 = max(0, angle2 - 1);
        servo2.write(angle2);
        break;
      case 'l':
        angle2 = min(180, angle2 + 5);
        servo2.write(angle2);
        break;
      case 'L':
        angle2 = min(180, angle2 + 1);
        servo2.write(angle2);
        break;
    }
    
    // Print current angles after every change
    Serial.print("shoulder: ");
    Serial.print(angle1);
    Serial.print(" | elbow: ");
    Serial.println(angle2);
  }
}
