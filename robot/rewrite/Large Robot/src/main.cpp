#include "robot-config.h"
#include "constants.h"
#define _USE_MATH_DEFINES
using namespace std;
#include <list>
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*        Description: Competition template for VCS VEX V5                   */
/*                                                                           */
/*---------------------------------------------------------------------------*/
#include "math.h" //Include math.h in order to gain access to math functions like PI.

//Creates a competition object that allows access to Competition methods.
vex::competition    Competition;

double flipperStartPosition;
double direction = 1;
void pre_auton( void ) {
  flipperStartPosition = Flipper.rotation(rotationUnits::deg)-180;
}

void unshoot() {
    FlyWheelMotorOne.stop();
    FlyWheelMotorTwo.stop();
    TopIntake.stop();
    BottomIntake.stop();
}
bool autonomousActive = false;

bool team = 1; //1 is red, -1 for blue
double xPosition = 0;
double yPosition = 0;
double orientation = 0;
bool driveType = false;
void forwardAutonomous(double distance, double speed) {
    LeftMotorOne.setVelocity(speed, velocityUnits::pct);
    LeftMotorTwo.setVelocity(speed, velocityUnits::pct);
    RightMotorOne.setVelocity(speed, velocityUnits::pct);
    RightMotorTwo.setVelocity(speed, velocityUnits::pct);
    //Converts linear distance to rotations of the wheel using formula
    //Then, rotates wheels accordingly
    double rotations = 360 * distance / wheelCircumference;
    LeftMotorOne.startRotateFor(rotations, rotationUnits::deg);
    LeftMotorTwo.startRotateFor(rotations, rotationUnits::deg);
    RightMotorOne.startRotateFor(-rotations, rotationUnits::deg);
    RightMotorTwo.rotateFor(-rotations, rotationUnits::deg);
    xPosition += sin(orientation * convertRadians) * distance;
    yPosition += cos(orientation * convertRadians) * distance;
}
void forwardAutonomous (double distance) {
  forwardAutonomous(distance, 30);
}
void turninplaceAutonomous(double degrees) {
    LeftMotorOne.setVelocity(30, velocityUnits::pct);
    LeftMotorTwo.setVelocity(30, velocityUnits::pct);
    RightMotorOne.setVelocity(30, velocityUnits::pct);
    RightMotorTwo.setVelocity(30, velocityUnits::pct);
    double robotWidth = robotWidthPos;
    //Creates an imaginary circle that is the circle of rotation
    //that the robot would rotate in. Calculates the circumference of that
    //circle and rotates the motors that number of degrees
    double rotations = degrees * robotWidth * team / wheelDiameter;
    LeftMotorOne.startRotateFor(rotations, rotationUnits::deg);
    LeftMotorTwo.startRotateFor(rotations, rotationUnits::deg);
    RightMotorOne.startRotateFor(rotations, rotationUnits::deg);
    RightMotorTwo.rotateFor(rotations, rotationUnits::deg);
    orientation += degrees;
    orientation = fmod(orientation, 360);
}
void rotateTo(double degrees) {
  turninplaceAutonomous(degrees - orientation);
  orientation = degrees;
}
void goTo(double x, double y) {
  double xDelta = x - xPosition;
  double yDelta = y - yPosition;
  rotateTo(atan(xDelta/yDelta) * convertDegrees);
  forwardAutonomous(sqrt(xDelta * xDelta + (yDelta * yDelta)));
}
void goTo(double x, double y, double degrees) {
  goTo(x, y);
  rotateTo(degrees);
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
void flip_helper(double amount) {
    Flipper.setVelocity(30, velocityUnits::pct);
    Flipper.rotateFor(amount, rotationUnits::deg);
    Flipper.stop(brakeType::hold);
}
void flip_autonomous() {
    flip_helper(110);
}
void unflip_autonomous() {
    flip_helper(-90);
}
void halt() {
    LeftLift.stop(brakeType::hold);
    RightLift.stop(brakeType::hold);
}
void lift_helper(double height) {
    halt();
    LeftLift.setVelocity(80, velocityUnits::pct);
    RightLift.setVelocity(80, velocityUnits::pct);
    LeftLift.startRotateTo(-height, rotationUnits::deg);
    RightLift.startRotateTo(height, rotationUnits::deg);
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
void place_ball() {
    TopIntake.spin(directionType::rev, 50, velocityUnits::pct);
}
void shoot_autonomous() {
    FlyWheelMotorOne.spin(directionType::rev, 100, velocityUnits::pct);
    FlyWheelMotorTwo.spin(directionType::rev, 100, velocityUnits::pct);
    while (FlyWheelMotorOne.velocity(velocityUnits::pct) > -95) {
        continue;
    }
    place_ball();
    vex::task::sleep(2000);
}
void place_ball_autonomous() {
    TopIntake.spin(directionType::rev, 100, velocityUnits::pct);
}
void spin_up() {
    FlyWheelMotorOne.spin(directionType::rev, 100, velocityUnits::pct);
    FlyWheelMotorTwo.spin(directionType::rev, 100, velocityUnits::pct);
}
void unspin_up() {
    FlyWheelMotorOne.stop();
    FlyWheelMotorTwo.stop();
}
void flipGround() {
    Flipper.startRotateTo(flipperStartPosition, rotationUnits::deg);
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
void autonomous( void ) {
    /*
    intake();
    forwardAutonomous(44);
    forwardAutonomous(-13);
    turninplaceAutonomous(-180 * team);
    unflip_autonomous();
    unflip_autonomous();
    forwardAutonomous(-13);
    flip_helper(40);
    //lift_helper(200);
    //vex::task::sleep(500);
    stop_intake();
    turninplaceAutonomous(180 * team);
    high_post_autonomous();
    forwardAutonomous(-(44 - 4));
    turninplaceAutonomous(-90 * team);
    forwardAutonomous(-(23.5/2.0));
    turninplaceAutonomous(90 * team);
    forwardAutonomous(-(2));
    lift_helper(1500);
    flip_helper(-30);
    vex::task::sleep(3000);
    flip_helper(-10);
    forwardAutonomous(2);
    floor_autonomous();
    forwardAutonomous(20);*/
    goTo(10, 10);
    /*
    spin_up();
    forwardAutonomous(29);
    shoot_autonomous();

    unflip_autonomous();
    unflip_autonomous();
    forwardAutonomous(-29);
    forwardAutonomous(-5);
    unshoot();

    turninplaceAutonomous(90 * team);
    intake();
    forwardAutonomous(42.75);
    turninplaceAutonomous(-90 * team);
    forwardAutonomous(-15);
    flip_autonomous();
    forwardAutonomous(4.5);//Changed from 4 -> 4.5

    mid_post_autonomous();
    turninplaceAutonomous(90 * team);
    forwardAutonomous(-48.25);//This value is countered by line 275. Changed from 49.25 -> 48.25

    lift_helper(2500);
    unflip_autonomous();
    forwardAutonomous(4.25);*/

}
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
        double SENSITIVITY_CONSTANT = -0.35;
        if (Controller1.ButtonR1.pressing()) {
            SENSITIVITY_CONSTANT = -1;
        }
        double linearPower = Controller1.Axis3.position(percentUnits::pct) * direction;
        double angularPower = Controller1.Axis1.value() * SENSITIVITY_CONSTANT;
        if (abs(linearPower) < 10) {
            linearPower = 0;
        }
        if (abs(angularPower) < 1) {
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
        if (abs(rightPower) < 10) {
            rightPower = 0;
        }
        LeftMotorOne.spin(directionType::fwd, leftPower, velocityUnits::pct);
        RightMotorOne.spin(directionType::rev, rightPower, velocityUnits::pct);
        LeftMotorTwo.spin(directionType::fwd, leftPower, velocityUnits::pct);
        RightMotorTwo.spin(directionType::rev, rightPower, velocityUnits::pct);
    }
}

int HIGHHIGH = 6000;
int HIGHMID = 3500;
int HIGH = 4500;
int MID = 3000;
int FLOOR = 0;
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
            if (abs(RightLift.rotation(rotationUnits::deg) - HIGHMID) <100) {
                power = 0;
            }
            else if (RightLift.rotation(rotationUnits::deg) < HIGHMID) {
                power = 80;
            } else {
                power = -80;
            }
        } else if (Controller2.ButtonX.pressing()) {
            if (abs(RightLift.rotation(rotationUnits::deg) - HIGH) <100) {
                power = 0;
            }
            else  if (RightLift.rotation(rotationUnits::deg) < HIGH) {
                power = 80;
            } else {
                power = -80;
            }
        } else if (Controller2.ButtonA.pressing()) {
            if (abs(RightLift.rotation(rotationUnits::deg) - MID) <100) {
                power = 0;
            }
            else if (RightLift.rotation(rotationUnits::deg) < MID) {
                power = 80;
            } else {
                power = -80;
            }
        } else if (Controller2.ButtonB.pressing()) {
            if (abs(RightLift.rotation(rotationUnits::deg) - FLOOR) <100) {
                power = 0;
            }
            else if (RightLift.rotation(rotationUnits::deg) < FLOOR) {
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
        double power = Controller2.Axis3.position(percentUnits::pct);
        if (Controller2.ButtonUp.pressing()) {
            power = 25;
        } else if (Controller2.ButtonDown.pressing()) {
            power = -25;
        } else if (Controller2.ButtonLeft.pressing()) {
            power = (flipperStartPosition - Flipper.rotation(rotationUnits::deg))/1.4;
        } else if (Controller2.ButtonRight.pressing()) {
            power = (flipperStartPosition + 115 - Flipper.rotation(rotationUnits::deg))/1.2;
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


double speed = 100;
void shoot(){
    //calculatePower(getDistance());
    FlyWheelMotorOne.spin(directionType::rev, speed, velocityUnits::pct);
    FlyWheelMotorTwo.spin(directionType::rev, speed, velocityUnits::pct);
    intake();
}
void stopPlace(){
  TopIntake.stop();
}
void incSpeed(){
  speed += 1;
}
void decSpeed(){
  speed -= 1;
}
int counter = 0;
void usercontrol( void ) {
  // User control code here, inside the loop
  while (1){
      //Start Controller 1
      driveNormal();
      if (Controller1.ButtonR1.pressing()){
              place_ball();
      }
      Controller1.ButtonR1.released(stopPlace);
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
      
      Controller1.ButtonRight.released(incSpeed);
      Controller1.ButtonLeft.released(decSpeed);
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
      
      counter++;
      if (counter % 10 == 0) {
        counter = 1;
        Controller1.Screen.print(-FlyWheelMotorOne.velocity(velocityUnits::rpm));
      }
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
