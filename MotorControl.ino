/*
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2
---->	http://www.adafruit.com/products/1438
*/

#include <Adafruit_MotorShield.h>

Adafruit_MotorShield laser_motors (0x60); // Default address, no jumpers
Adafruit_MotorShield mirror_motors (0x61); // Rightmost jumper closed
Adafruit_MotorShield camera_motor (0x62); // second right most jumper closed

// On the bottom shield, connect two steppers, each with 200 steps
Adafruit_StepperMotor *laser_left = laser_motors.getStepper(200, 1); // left eye
Adafruit_StepperMotor *laser_right = laser_motors.getStepper(200, 2); // right eye

// On the middle shield connect a stepper to port M3/M4 with 200 steps
Adafruit_StepperMotor *mirror_left = mirror_motors.getStepper(200, 1); // left eye
Adafruit_StepperMotor *mirror_right = mirror_motors.getStepper(200, 2); // right eye

// On the top shield connect a stepper to port M3/M4 with 200 steps
Adafruit_StepperMotor *camera = camera_motor.getStepper(200, 1);

const byte numChars = 32;
char receivedChars[numChars];

void setup() {
  while (!Serial);
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.setTimeout(1);
  Serial.println("<Arduino is ready>");

  laser_motors.begin(); // Start the bottom shield
  mirror_motors.begin(); // Start the middle shield
  camera_motor.begin(); // Start the top shield

  laser_left->setSpeed(1);  // 1 rpm
  laser_right->setSpeed(1);

  mirror_left->setSpeed(1);  // 1 rpm
  mirror_right->setSpeed(1);

  camera->setSpeed(1);  // 1 rpm
}

int i;
bool left = true;
void loop() {
  while (!Serial.available());
  String msg; 
  msg = recvWithEndMarker();
  Serial.println(msg + "\n");
  if (msg.equals("L")) {
      left = true; 
      Serial.println("inside left ");
      return;
  } else if (msg.equals("R")) {
      left = false;
      return;
  } else {
    moveComponents(left, msg);
  }


  // while (Serial.available() > 0) {
  //   String rc;
  //   rc = Serial.readStringUntil('\n');
  //   Serial.println(rc);
  //   if (rc.equals("L\n")) {
  //     left = true; 
  //     Serial.println("inside left");
  //   } else if (rc.equals("R\n")) {
  //     left = false;
  //   }

  //   command = Serial.readStringUntil('\n');
  //   moveComponents(left, command);
  // }
  //   for (i=0; i<255; i++) {
  //     // myStepper1->onestep(FORWARD, INTERLEAVE);
  //     // myStepper2->onestep(BACKWARD, DOUBLE);
  //     // myStepper3->onestep(FORWARD, MICROSTEP);
  //     delay(3);
  //  }

  //  for (i=255; i!=0; i--) {
  //     // myStepper1->onestep(BACKWARD, INTERLEAVE);
  //     // myStepper2->onestep(FORWARD, DOUBLE);
  //     // myStepper3->onestep(BACKWARD, MICROSTEP);
  //     delay(3);
  //  }

  //   for (i=0; i<255; i++) {
  //     // myStepper1->onestep(FORWARD, DOUBLE);
  //     // myStepper2->onestep(BACKWARD, INTERLEAVE);
  //     // myStepper3->onestep(FORWARD, MICROSTEP);
  //     delay(3);
  //  }

  //   for (i=255; i!=0; i--) {
  //     // myStepper1->onestep(BACKWARD, DOUBLE);
  //     // myStepper2->onestep(FORWARD, INTERLEAVE);
  //     // myStepper3->onestep(BACKWARD, MICROSTEP);
  //     delay(3);
  //  }
  }

  String recvWithEndMarker() {
    char rc;
    char endMarker = '\n';
    static byte ndx = 0;
    while (Serial.available() > 0) {
      rc = Serial.read();
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
      } else {
        receivedChars[ndx] = '\0';
        ndx = 0;
        static byte ndx = 0;
        char rc;
      }
    }
    return String(rc);
  }

  void moveComponents(boolean left, String cmd) {
    Serial.println(cmd + "\n");
    if (cmd.equals("\0")) {
        return;
    } else {
        if (cmd.equals("A")) {
          // camera->step(5, BACKWARD, DOUBLE);
          camera->onestep(FORWARD, DOUBLE);
      } else if (cmd.equals("B")) {
          camera->onestep(BACKWARD, DOUBLE);
      } else if (cmd.equals("C")) {
        if (left) {
          mirror_left->onestep(FORWARD, SINGLE);
        } else {
          mirror_right->onestep(FORWARD, SINGLE);
        }
      } else if (cmd.equals("D")) {
        if (left) {
          mirror_left->onestep(BACKWARD, SINGLE);
        } else {
          mirror_right->onestep(BACKWARD, SINGLE);
        }
      } else if (cmd.equals("E")) {
        if (left) {
          laser_left->onestep(FORWARD, SINGLE);
          Serial.println("inside laser");
        } else {
          laser_right->onestep(FORWARD, SINGLE);
        } 
      } else if (cmd.equals("F")) {
        if (left) {
          laser_left->onestep(BACKWARD, SINGLE);
        } else {
          laser_right->onestep(BACKWARD, SINGLE);
        }
      }
    }
 }