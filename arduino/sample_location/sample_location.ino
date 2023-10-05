#include <Servo.h>

Servo shoulder;
Servo elbow;

struct robot_state {
  int shoulder_angle;
  int elbow_angle;
};

template <typename T>
struct optional {
  T value;
  bool valid;
};

optional<robot_state> get_command(char c) {
  switch (c) {
    case '0':
      return {{135, 94}, true}; //
    case '1':
      return {{96, 49}, true}; //
    case '2':
      return {{66, 29}, true};//
    case '3':
      return {{177, 154}, true}; //
    case '4':
      return {{176, 175}, true};//
    case '5':
      return {{52, 0}, true};
     case 'r':
      return {{90, 90}, true};
    default:
      return {{0, 0}, false};
  }
}

void send_command(robot_state const& command){
  shoulder.write(command.shoulder_angle);
  elbow.write(command.elbow_angle);
}

bool read_sensor() {
  auto const value = digitalRead(2);
  return value == 0 ? true : false;
}

void setup() {
  //start serial connection
  Serial.begin(9600);
  //configure pin 2 as an input and enable the internal pull-up resistor
  pinMode(2, INPUT_PULLUP);
  pinMode(13, OUTPUT);

  shoulder.attach(9);
  elbow.attach(10);
}

void loop() { 
  // Only respond to new commands
  if (!Serial.available()) {
    return;
  }
  // parse the command to joint state
  auto const joint_state = get_command(Serial.read());
  if (!joint_state.valid) {
    return;
  }
  send_command(joint_state.value);

  // allow arm time to travel
  delay(1000);

  auto const detected = read_sensor();
 
  if (detected) {
    digitalWrite(13, HIGH);
    Serial.println(1);
    return;
  }
  
  digitalWrite(13, LOW);
  Serial.println(0);

}
