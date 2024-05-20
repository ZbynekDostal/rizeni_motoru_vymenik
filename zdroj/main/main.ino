/*
    Controlling two stepper with the AccelStepper library

     by Dejan, https://howtomechatronics.com

     example: https://howtomechatronics.com/tutorials/arduino/stepper-motors-and-arduino-the-ultimate-guide/
*/

#include <AccelStepper.h>

// Define the stepper motor and the pins that is connected to
AccelStepper stepper1(1, 9, 8); // (Typeof driver: with 2 pins, STEP, DIR)

void setup() {

  stepper1.setMaxSpeed(6000); // Set maximum speed value for the stepper
  stepper1.setAcceleration(50000); // Set acceleration value for the stepper
  stepper1.setCurrentPosition(0); // Set the current position to 0 steps
}

void loop() {

  stepper1.moveTo(1000); // Set desired move: 800 steps (in quater-step resolution that's one rotation)
  stepper1.runToPosition(); // Moves the motor to target position w/ acceleration/ deceleration and it blocks until is in position

  // Move back to position 0, using run() which is non-blocking - both motors will move at the same time
  stepper1.moveTo(0);
  stepper1.runToPosition(); // Moves the motor to target position w/ acceleration/ deceleration and it blocks until is in position

}