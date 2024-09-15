#include <Servo.h>

int const READ_SENSOR = 1000;
int const SHOULDER_ZERO = 86;
int const ELBOW_ZERO = 90;
double const SHOULDER_SCALE = 0.75;
double const ELBOW_SCALE = 0.7;

Servo shoulder;
Servo elbow;

struct robot_state {
  double shoulder_angle;
  double elbow_angle;
};

template <typename T>
struct optional {
  T value;
  bool valid;
};

optional<robot_state> get_command(Serial_& serial) {
  robot_state state{0, 0};

  // Read the incoming string until a newline character
  String const input = serial.readStringUntil('\n');
  // Parse the joint angles from the input string
  int const comma_index = input.indexOf(',');
  if (comma_index > 0) {
    String shoulder_str = input.substring(0, comma_index);
    String elbow_str = input.substring(comma_index + 1);

    shoulder_str.trim();
    elbow_str.trim();

    state.shoulder_angle = shoulder_str.toDouble();
    state.elbow_angle = elbow_str.toDouble();
    return {state, true};
  }
  return {{0, 0}, false};
}

bool read_sensor() {
  auto const value = digitalRead(2);
  return (value == LOW);
}

void send_command(robot_state const& command){
  shoulder.write(SHOULDER_ZERO + int(SHOULDER_SCALE * command.shoulder_angle));
  elbow.write(ELBOW_ZERO + int(ELBOW_SCALE * command.elbow_angle));
}

void send_command_mock(robot_state const& command){
  Serial.print("[");
  Serial.print(command.shoulder_angle);
  Serial.print(", ");
  Serial.print(command.elbow_angle);
  Serial.println("]");
}

void setup() {
  // Start serial communication
  Serial.begin(9600);

  // Initialize servos
  shoulder.attach(9);
  elbow.attach(10);
  // Send to zero position
  shoulder.write(SHOULDER_ZERO);
  elbow.write(ELBOW_ZERO);

  // Configure pin 2 as an input for the sensor and enable the internal pull-up resistor
  pinMode(2, INPUT_PULLUP);
  pinMode(13, OUTPUT); // LED indicator (optional)
  digitalWrite(13, LOW); // Turn off LED
}

void loop() {
  // Check if data is available on the serial port
  if (!Serial.available()) {
    return;
  }

  auto const command_maybe = get_command(Serial);
  if (!command_maybe.valid) {
    return;
  }
  if (command_maybe.value.shoulder_angle == READ_SENSOR) {
    // Read sensor and send the result back
    bool detected = read_sensor();
    if (detected) {
      digitalWrite(13, HIGH); // Turn on LED
      Serial.println(1);
      return;
    }

    digitalWrite(13, LOW); // Turn off LED
    Serial.println(0);
    return;
  }
  // Send commands to the servos
  // send_command_mock(command_maybe.value);
  send_command(command_maybe.value);

  // Allow time for the servos to reach the position
  delay(50);
}


/*
#include <Servo.h>

Servo shoulder;
Servo elbow;

void setup() {
  // Start serial communication
  Serial.begin(9600);

  // Initialize servos
  shoulder.attach(9);
  elbow.attach(10);

  // Configure pin 2 as an input for the sensor and enable the internal pull-up resistor
  pinMode(2, INPUT_PULLUP);
  pinMode(13, OUTPUT); // LED indicator (optional)
}

void loop() {
  // Check if data is available on the serial port
  if (Serial.available()) {
    // Read the incoming string until a closing brace '}'
    String input = Serial.readStringUntil('}');

    // Ensure that the input starts with '{'
    int startIndex = input.indexOf('{');
    if (startIndex >= 0) {
      // Extract the content inside the braces
      String content = input.substring(startIndex + 1);

      // Parse the joint angles from the content
      int commaIndex = content.indexOf(',');
      if (commaIndex > 0) {
        String shoulder_str = content.substring(0, commaIndex);
        String elbow_str = content.substring(commaIndex + 1);

        // Trim any whitespace
        shoulder_str.trim();
        elbow_str.trim();

        // Convert strings to integers
        int shoulder_angle = shoulder_str.toInt();
        int elbow_angle = elbow_str.toInt();

        // Send commands to the servos
        send_command(shoulder_angle, elbow_angle);

        // Allow time for the servos to reach the position
        delay(1000);

        // Read sensor and send the result back
        bool detected = read_sensor();
        if (detected) {
          digitalWrite(13, HIGH); // Turn on LED
          Serial.println(1);
        } else {
          digitalWrite(13, LOW); // Turn off LED
          Serial.println(0);
        }
      }
    }
  }
}

void send_command(int shoulder_angle, int elbow_angle) {
  shoulder.write(shoulder_angle);
  elbow.write(elbow_angle);
}

bool read_sensor() {
  int value = digitalRead(2);
  return (value == LOW); // Assuming active LOW sensor
}
*/
