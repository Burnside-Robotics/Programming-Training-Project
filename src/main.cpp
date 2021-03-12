/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;
using namespace std;
// A global instance of competition
competition Competition;
controller Controller1;

motor lDrive1(PORT1, ratio18_1);
motor lDrive2(PORT2, ratio18_1);
motor rDrive1(PORT10, ratio18_1, true);
motor rDrive2(PORT6, ratio18_1, true);

motor_group lDrive(lDrive1, lDrive2);
motor_group rDrive(rDrive1, rDrive2);

motor leftArm(PORT10, ratio36_1);
motor rightArm(PORT11, ratio36_1, true);

motor_group Arm(leftArm, rightArm);

motor Claw(PORT20, ratio36_1);

float ArmSpeed = 100;
float ClawSpeed = 100;


const float WHEEL_CIRCUMFERENCE = 31.9185813596;

//Control the Drive Motors
void MoveMotors(float lInput, float rInput)
{
  lDrive.spin(fwd, lInput, pct);
  rDrive.spin(fwd, rInput, pct);
}

// define your global instances of motors and other devices here

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

//Drive the Robot a certain Distance
void DriveDistance(int dist, float maxTime)
{
  lDrive1.resetPosition();
  rDrive1.resetPosition();

  //Constant Tuning Values
  const float Kp = 1;
  const float Ki = 0;
  const float Kd = 0;

  float rotationGoal = (dist / WHEEL_CIRCUMFERENCE) * 360;

  const float maxSpeed = 100;
  const float accelTime = 500;
  

  float distError = 0;
  float integral = 0;
  float derivative = 0;
  float lastError = 0;

  float currentMaxSpeed = 0;
  float motorSpeed = 0; 
  

  float dunnTime = 0;
  while(maxTime > dunnTime / 1000)
  {
    distError = rotationGoal - lDrive1.rotation(deg);

    integral += distError;

    if(distError > 200 || distError < -200)
    {
      integral = 0;
    }

    derivative = distError - lastError;

    lastError = distError;

    motorSpeed = Kp * distError + Ki * integral + Kd * derivative;

    if(dunnTime < accelTime)
    {
      if(dist < 0)
      {
        currentMaxSpeed = -(dunnTime / (accelTime / maxSpeed));
      }
      else 
      {
        currentMaxSpeed = dunnTime / (accelTime / maxSpeed);
      }
    }
    if(dist > 0)
    {
      motorSpeed = currentMaxSpeed < motorSpeed ? currentMaxSpeed : motorSpeed;
    }
    else 
    {
      motorSpeed = currentMaxSpeed > motorSpeed ? currentMaxSpeed : motorSpeed;
    }
    lDrive.spin(fwd, motorSpeed, pct);
    rDrive.spin(fwd, motorSpeed, pct);

    wait(15, msec);
    dunnTime += 15;
  }
}
void autonomous(void) {
  DriveDistance(50, 5);
}

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    MoveMotors(Controller1.Axis3.value(), Controller1.Axis2.value());

    //Control Lift
    if(Controller1.ButtonL1.pressing())
      Arm.spin(fwd, ArmSpeed, pct);
    else if(Controller1.ButtonL2.pressing()) 
      Arm.spin(fwd, -ArmSpeed, pct);
    else
      Arm.stop(hold);

    //Control Claw
     if(Controller1.ButtonR1.pressing())
      Claw.spin(fwd, ClawSpeed, pct);
    else if(Controller1.ButtonR2.pressing()) 
      Claw.spin(fwd, -ClawSpeed, pct);
    else
      Claw.stop(hold);

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
