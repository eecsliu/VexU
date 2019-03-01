#include "robot-config.h"
#define _USE_MATH_DEFINES
using namespace std;
#include <list>;
#include <cmath>        // std::abs
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*        Description: Competition template for VCS VEX V5                   */
/*                                                                           */
/*---------------------------------------------------------------------------*/
#include "math.h" //Include math.h in order to gain access to math functions like PI.

//Creates a competition object that allows access to Competition methods.
vex::competition    Competition;

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the cortex has been powered on and    */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/
double flipperStartPosition;
double direction = 1;
void pre_auton( void ) {
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  flipperStartPosition = Flipper.rotation(rotationUnits::deg)-180;
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void unshoot() {
    FlyWheelMotor.stop();
    TopIntake.stop();
    BottomIntake.stop();
}
bool autonomousActive = false;
const double wheelDiameter = 4;
const double wheelCircumference = wheelDiameter * M_PI;
const double robotWidth = 11.25 * (360 / (360 - 16.399));
bool driveType = false;
void forwardAutonomous(double distance) {
    autonomousActive = true;
    LeftMotorOne.setVelocity(20, velocityUnits::pct);
    RightMotorOne.setVelocity(20, velocityUnits::pct);
    LeftMotorTwo.setVelocity(20, velocityUnits::pct);
    RightMotorTwo.setVelocity(20, velocityUnits::pct);
    LeftMotorOne.startRotateFor(360 * distance / wheelCircumference, rotationUnits::deg);
    LeftMotorTwo.startRotateFor(360 * distance / wheelCircumference, rotationUnits::deg);
    RightMotorOne.startRotateFor(-360 * distance / wheelCircumference, rotationUnits::deg);
    RightMotorTwo.rotateFor(-360 * distance / wheelCircumference, rotationUnits::deg);
    autonomousActive = false;
    vex::task::sleep(300);
    //vexGenericSerialEnable( vex::PORT18, 0 );
    //https://www.vexforum.com/index.php/34239-vex-u?search=serial
}
void climb() {
    double distance = 45;
    LeftMotorOne.setVelocity(50, velocityUnits::pct);
    RightMotorOne.setVelocity(50, velocityUnits::pct);
    LeftMotorTwo.setVelocity(50, velocityUnits::pct);
    RightMotorTwo.setVelocity(50, velocityUnits::pct);
    LeftMotorOne.startRotateFor(360 * distance / wheelCircumference, rotationUnits::deg);
    LeftMotorTwo.startRotateFor(360 * distance / wheelCircumference, rotationUnits::deg);
    RightMotorOne.startRotateFor(-360 * distance / wheelCircumference, rotationUnits::deg);
    RightMotorTwo.rotateFor(-360 * distance / wheelCircumference, rotationUnits::deg);
}
/*
bool receiveSerial() {
    return !Ready.pressing();
}

void serialTank() {
    LeftMotor.setVelocity(20, velocityUnits::pct);
    RightMotor.setVelocity(20, velocityUnits::pct);
    if (LeftSerial.pressing()) {
        LeftMotor.spin(directionType::fwd);
    }else {
        LeftMotor.spin(directionType::rev);
    }
    if (RightSerial.pressing()) {
        RightMotor.spin(directionType::rev);
    }else {
        RightMotor.spin(directionType::fwd);
    }
}*/

void shoot_autonomous() {
    FlyWheelMotor.spin(directionType::rev, 10, velocityUnits::pct);
    BottomIntake.spin(directionType::fwd, 10, velocityUnits::pct);
    vex::task::sleep(200);
    FlyWheelMotor.stop();
    BottomIntake.stop();
    /*
    if (Running.pressing()) {
        while (receiveSerial()) {
            serialTank();
        }
    }*/
    FlyWheelMotor.spin(directionType::fwd, 100, velocityUnits::pct);
    while(FlyWheelMotor.velocity(velocityUnits::pct) < 95) {
        continue;
    }
    if(FlyWheelMotor.velocity(velocityUnits::pct)>= 95) {
        BottomIntake.spin(directionType::rev, 50, velocityUnits::pct);
    }
    vex::task::sleep(300);
    unshoot();
}

void turninplaceAutonomous(double degrees) {
    autonomousActive = true;
    LeftMotorOne.setVelocity(20, velocityUnits::pct);
    RightMotorOne.setVelocity(20, velocityUnits::pct);
    LeftMotorTwo.setVelocity(20, velocityUnits::pct);
    RightMotorTwo.setVelocity(20, velocityUnits::pct);
    LeftMotorOne.startRotateFor(degrees * robotWidth / wheelDiameter, rotationUnits::deg);
    LeftMotorTwo.startRotateFor(degrees * robotWidth / wheelDiameter, rotationUnits::deg);
    RightMotorOne.startRotateFor(degrees * robotWidth / wheelDiameter, rotationUnits::deg);
    RightMotorTwo.rotateFor(degrees * robotWidth / wheelDiameter, rotationUnits::deg);
    autonomousActive = false;
}

void place_ball() {
    TopIntake.spin(directionType::rev, 50, velocityUnits::pct);
}
void place_ball_autonomous() {
    TopIntake.spin(directionType::rev, 100, velocityUnits::pct);
}
void spin_up() {
    FlyWheelMotor.spin(directionType::rev, 100, velocityUnits::pct);
}
void unspin_up() {
    FlyWheelMotor.stop();
}
void flipGround() {
    Flipper.startRotateTo(flipperStartPosition, rotationUnits::deg);
}

void autonomous( void ) {
    int team = 1; //1 for red, -1 for blue
    forwardAutonomous(8);
    //shoot_autonomous();
    forwardAutonomous(38);
    forwardAutonomous(-12);
    turninplaceAutonomous(90 * team);
    forwardAutonomous(47);
    turninplaceAutonomous(-90 * team);
    forwardAutonomous(12);
    forwardAutonomous(-12);
    turninplaceAutonomous(-90 * team);
    forwardAutonomous(12);
    turninplaceAutonomous(-90 * team);
    forwardAutonomous(25);
    climb();
    //Start of robot skills. BE SURE TO COMMENT OUT DURING COMPETITION
    /*
    forwardAutonomous(6);
    turninplaceAutonomous(-90 * team);
    forwardAutonomous(12);
    turninplaceAutonomous(-90);
    forwardAutonomous(52);
    turninplaceAutonomous(-90 * team);
    forwardAutonomous(18);
    forwardAutonomous(-4);
    turninplaceAutonomous(-90 * team);
    */
    //End of robot skills

}
/*----------------------------------------------------------------------------*/
/*                                                                            */
/*                              User Control Task                             */
/*                                                                            */
/*  This task is used to control your robot during the user control phase of  */
/*  a VEX Competition.                                                        */
/*                                                                            */
/*  You must modify the code to add your own robot specific commands here.    */
/*----------------------------------------------------------------------------*/
void activate_motors(double angularPower, double linearPower){
    double rightPwm = linearPower;
    double leftPwm = linearPower;
    leftPwm -= angularPower;
    rightPwm += angularPower;
    if (leftPwm > 100){
        rightPwm -= leftPwm - 100;
        leftPwm = 100;
    }
    else if (rightPwm > 100){
        leftPwm -= rightPwm - 100;
        rightPwm = 100;
    }
    else if (leftPwm < -100){
        rightPwm += -100 - leftPwm;
        leftPwm = -100;
    }
    else if (rightPwm < -100){
        leftPwm += -100 - rightPwm;
        rightPwm = -100;
    }
    if (leftPwm < 0) {
        LeftMotorOne.spin(directionType::rev, - leftPwm, velocityUnits::pct);
        LeftMotorTwo.spin(directionType::rev, - leftPwm, velocityUnits::pct);
    }
    else {
        LeftMotorOne.spin(directionType::fwd, leftPwm, velocityUnits::pct);
        LeftMotorTwo.spin(directionType::fwd, leftPwm, velocityUnits::pct);
    }
    if (rightPwm < 0) {
        RightMotorOne.spin(directionType::fwd, -rightPwm, velocityUnits::pct);
        RightMotorTwo.spin(directionType::fwd, -rightPwm, velocityUnits::pct);
    }
    else {
        RightMotorOne.spin(directionType::rev, rightPwm, velocityUnits::pct);
        RightMotorTwo.spin(directionType::rev, rightPwm, velocityUnits::pct);
    }
}
void driveNormal(){
    if (autonomousActive == false){
        double SENSITIVITY_CONSTANT = -0.15;
        if (Controller1.ButtonR1.pressing()) {
            SENSITIVITY_CONSTANT = -1;
        }
        double linearPower = Controller1.Axis3.position(percentUnits::pct) * direction;
        double angularPower = Controller1.Axis1.value() * SENSITIVITY_CONSTANT;
        if (abs(linearPower) < 10) {
            linearPower = 0;
        }
        if (abs(angularPower < 10)) {
            angularPower = 0;
        }
        activate_motors(angularPower, linearPower);
    }
}
void driveTank(){
    if (autonomousActive == false) {
        double leftPower = Controller1.Axis3.position(percentUnits::pct);
        double rightPower = Controller1.Axis2.position(percentUnits::pct);
        if (abs(leftPower) < 10) {
            leftPower = 0;
        }
        if (abs(rightPower < 10)) {
            rightPower = 0;
        }
        LeftMotorOne.spin(directionType::fwd, leftPower, velocityUnits::pct);
        RightMotorOne.spin(directionType::rev, rightPower, velocityUnits::pct);
        LeftMotorTwo.spin(directionType::fwd, leftPower, velocityUnits::pct);
        RightMotorTwo.spin(directionType::rev, rightPower, velocityUnits::pct);
    }
}
void toggleDrive(){
    driveType = !driveType;
    Controller1.Screen.print(driveType);
}

void toggleShooterFront() {
    direction = 1;
}

void toggleLifterFront() {
    direction = -1;
}

void toggleNull() {
    return;
}

void unjamShooter() {
    TopIntake.spin(directionType::fwd, 50, velocityUnits::pct);
}
void stopUnjam() {
    TopIntake.stop();
}

void intake(){
    BottomIntake.spin(directionType::rev, 50, velocityUnits::pct);
}
void reverse_intake() {
    BottomIntake.spin(directionType::fwd, 100, velocityUnits::pct);
}

void stop_intake() {
    BottomIntake.stop();
}
void maintain_flip() {
    Flipper.stop(brakeType::hold);
}
void halt() {
    LeftLift.stop(brakeType::hold);
    RightLift.stop(brakeType::hold);
}

void lift(){
    if (autonomousActive == false){
        double power = Controller2.Axis2.position(percentUnits::pct);
        if (Controller2.ButtonR2.pressing()) {
            if (abs(RightLift.rotation(rotationUnits::deg) - 6000) <100) {
                power = 0;
            }
            else if (RightLift.rotation(rotationUnits::deg) < 6000) {
                power = 80;
            } else {
                power = -80;
            }
        } else if (Controller2.ButtonR1.pressing()) {
            if (abs(RightLift.rotation(rotationUnits::deg) - 5000) <100) {
                power = 0;
            }
            else if (RightLift.rotation(rotationUnits::deg) < 5000) {
                power = 80;
            } else {
                power = -80;
            }
        } else if (Controller2.ButtonX.pressing()) {
            if (abs(RightLift.rotation(rotationUnits::deg) - 4500) <100) {
                power = 0;
            }
            else  if (RightLift.rotation(rotationUnits::deg) < 4500) {
                power = 80;
            } else {
                power = -80;
            }
        } else if (Controller2.ButtonA.pressing()) {
            if (abs(RightLift.rotation(rotationUnits::deg) - 3000) <100) {
                power = 0;
            }
            else if (RightLift.rotation(rotationUnits::deg) < 3000) {
                power = 80;
            } else {
                power = -80;
            }
        } else if (Controller2.ButtonB.pressing()) {
            if (abs(RightLift.rotation(rotationUnits::deg) - 0) <100) {
                power = 0;
            }
            else if (RightLift.rotation(rotationUnits::deg) < 0) {
                power = 80;
            } else {
                power = -80;
            }
        }
        if (abs(power) < 10) {
            halt();
            return;
        }
        if (power < 0) {
            LeftLift.spin(directionType::fwd, -power, percentUnits::pct);
            RightLift.spin(directionType::rev, -power, percentUnits::pct);
        } else {
            LeftLift.spin(directionType::rev, power, percentUnits::pct);
            RightLift.spin(directionType::fwd, power, percentUnits::pct);
        }
    }
}
void flip() {
    if (autonomousActive == false){
        double power = Controller2.Axis3.position(percentUnits::pct)/2;
        if (Controller2.ButtonUp.pressing()) {
            power = 50;
        } else if (Controller2.ButtonDown.pressing()) {
            power = -50;
        } else if (Controller2.ButtonLeft.pressing()) {
            power = (flipperStartPosition - Flipper.rotation(rotationUnits::deg))/2.0;
        } else if (Controller2.ButtonRight.pressing()) {
            power = (flipperStartPosition + 105 - Flipper.rotation(rotationUnits::deg))/1.2;
        }
        if (abs(power) < 10) {
            maintain_flip();
            return;
        }
        if (power < 0) {
            Flipper.spin(directionType::rev, -power, percentUnits::pct);
        } else {
            Flipper.spin(directionType::fwd, power, percentUnits::pct);
        }
    }
}

void lift_helper(double height) {
    halt();
    LeftLift.setVelocity(80, velocityUnits::pct);
    RightLift.setVelocity(80, velocityUnits::pct);
    LeftLift.startRotateTo(height, rotationUnits::deg);
    RightLift.startRotateTo(-height, rotationUnits::deg);
}
void high_post_autonomous() {
    lift_helper(4000);
}
void mid_post_autonomous() {
    lift_helper(3000);
}
void floor_autonomous() {
    halt();
    lift_helper(0);
}

void shoot(){
    //calculatePower(getDistance());
    FlyWheelMotor.spin(directionType::rev, 100, velocityUnits::pct);
    intake();
}

void usercontrol( void ) {
  // User control code here, inside the loop
  while (1){
      //Start Controller 1
      driveNormal();
      if (Controller1.ButtonR2.pressing()){
        if (FlyWheelMotor.velocity(velocityUnits::pct) < -95) {
              place_ball();
        }
      }
      Controller1.ButtonUp.pressed(toggleShooterFront);
      Controller1.ButtonUp.released(toggleNull);

      Controller1.ButtonDown.pressed(toggleLifterFront);
      Controller1.ButtonDown.released(toggleNull);

      Controller1.ButtonL2.pressed(reverse_intake);
      Controller1.ButtonL2.released(stop_intake);

      Controller1.ButtonL1.pressed(intake);
      Controller1.ButtonL1.released(stop_intake);

      Controller1.ButtonR2.pressed(shoot);
      Controller1.ButtonR2.released(unshoot);

      Controller1.ButtonB.pressed(unjamShooter);
      Controller1.ButtonB.released(stopUnjam);

      //Start Controller 2
      lift();
      flip();

      Controller2.ButtonA.pressed(mid_post_autonomous);
      Controller2.ButtonA.released(toggleNull);

      Controller2.ButtonB.pressed(floor_autonomous);
      Controller2.ButtonB.released(toggleNull);

      Controller2.ButtonX.pressed(high_post_autonomous);
      Controller2.ButtonX.released(toggleNull);

      Controller2.ButtonLeft.pressed(flipGround);
      Controller2.ButtonLeft.released(maintain_flip);

      Controller2.ButtonL1.pressed(spin_up);
      Controller2.ButtonL1.released(unspin_up);

    vex::task::sleep(20); //Sleep the task for a short amount of time to prevent wasted resources.
  }
}
//
// Main will set up the competition functions and callbacks.
//
int main() {

    //Run the pre-autonomous function.
    pre_auton();
    //vex::thread rxThread( receiveTask );
    //Set up callbacks for autonomous and driver control periods.
    Competition.autonomous( autonomous );
    Competition.drivercontrol( usercontrol );

    //Prevent main from exiting with an infinite loop.
    // vex::thread rxThread( receiveTask );
    while(1) {
      vex::task::sleep(100);//Sleep the task for a short amount of time to prevent wasted resources.
    }

}
