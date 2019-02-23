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
bool autonomousActive = false;
const double wheelDiameter = 4;
const double wheelCircumference = wheelDiameter * M_PI;
const double robotWidth = 11.25 * (360.0 / 340.56);
bool driveType = false;
void forwardAutonomous(double distance) {
    autonomousActive = true;
    LeftMotor.setVelocity(30, velocityUnits::pct);
    RightMotor.setVelocity(30, velocityUnits::pct);
    LeftMotor.startRotateFor(360 * distance / wheelCircumference, rotationUnits::deg);
    RightMotor.rotateFor(-360 * distance / wheelCircumference, rotationUnits::deg);
    autonomousActive = false;
}

void turninplaceAutonomous(double degrees) {
    autonomousActive = true;
    LeftMotor.setVelocity(30, velocityUnits::pct);
    RightMotor.setVelocity(30, velocityUnits::pct);
    LeftMotor.startRotateFor(degrees * robotWidth / wheelDiameter, rotationUnits::deg);
    RightMotor.rotateFor(degrees * robotWidth / wheelDiameter, rotationUnits::deg);
    autonomousActive = false;
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
    if (leftPwm < 0)
        LeftMotor.spin(directionType::rev, - leftPwm, velocityUnits::pct);
    else
        LeftMotor.spin(directionType::fwd, leftPwm, velocityUnits::pct);
    if (rightPwm < 0)
        RightMotor.spin(directionType::fwd, -rightPwm, velocityUnits::pct);
    else
        RightMotor.spin(directionType::rev, rightPwm, velocityUnits::pct);
}
void driveNormal(){
    if (autonomousActive == false){
        double SENSITIVITY_CONSTANT;
        if (Controller1.ButtonR1.pressing()) {
            SENSITIVITY_CONSTANT = -0.15;
        }
        else {
            SENSITIVITY_CONSTANT = -0.30;
        }
        double linearPower = Controller1.Axis3.position(percentUnits::pct) * direction;
        double angularPower = Controller1.Axis1.value() * SENSITIVITY_CONSTANT;
        activate_motors(angularPower, linearPower);
    }
}
void driveTank(){
    if (autonomousActive == false) {
        LeftMotor.spin(directionType::fwd, Controller1.Axis3.position(percentUnits::pct), velocityUnits::pct);
        RightMotor.spin(directionType::rev, Controller1.Axis2.position(percentUnits::pct), velocityUnits::pct);
    }
}
void toggleDrive(){
    direction = -direction;
    Controller1.Screen.print(driveType);
}

void toggleNull() {
    return;
}

void unshoot() {
    FlyWheelMotor.stop();

    TopIntake.stop();
}
void place_ball() {
    TopIntake.spin(directionType::rev, 50, velocityUnits::pct);
}
void place_ball_autonomous() {
    TopIntake.spin(directionType::rev, 100, velocityUnits::pct);
    while (TopIntake.velocity(velocityUnits::pct) < 95.0) {
        continue;
    }
    unshoot();
}
void spin_up() {
    FlyWheelMotor.spin(directionType::rev, 100, velocityUnits::pct);
}
void unspin_up() {
    FlyWheelMotor.stop();
}
void shoot(){
    //calculatePower(getDistance());
    if (!Controller2.ButtonR1.pressing()) {
        return;
    }
    FlyWheelMotor.spin(directionType::rev, 100, velocityUnits::pct);
    while (FlyWheelMotor.velocity(velocityUnits::pct) > -95 && Controller2.ButtonR1.pressing()) {
        continue;
    }
    place_ball();
}
void shoot_autonomous() {
    //calculatePower(getDistance());
    FlyWheelMotor.spin(directionType::rev, 100, velocityUnits::pct);
    while (FlyWheelMotor.velocity(velocityUnits::pct) > -95) {
        continue;
    }
    place_ball();
    vex::task::sleep(2000);
}
void intake(){
    BottomIntake.spin(directionType::rev, 50, velocityUnits::pct);
}

void reverse_intake() {
    BottomIntake.spin(directionType::fwd, 50, velocityUnits::pct);
}

void stop_intake() {
    BottomIntake.stop();
}

double max_rotation = 180;

void maintain_flip() {
    Flipper.stop(brakeType::hold);
}

void flip(){
    Flipper.spin(directionType::fwd, 30, velocityUnits::pct);
    while(Flipper.rotation(rotationUnits::deg) < 100 + flipperStartPosition && Controller2.ButtonUp.pressing()) {
        continue;
    }
    Flipper.stop(brakeType::hold);
}

void flip_autonomous() {
    Flipper.setVelocity(30, velocityUnits::pct);
    Flipper.rotateFor(90, rotationUnits::deg);
    Flipper.stop(brakeType::hold);
}
void unflip() {
    Flipper.spin(directionType::rev, 30, velocityUnits::pct);
    if(!(Flipper.rotation(rotationUnits::deg) > 5 + flipperStartPosition)) {
        Flipper.stop(brakeType::hold);
    }
}
void unflip_autonomous() {
    Flipper.setVelocity(30, velocityUnits::pct);
    Flipper.rotateFor(-90, rotationUnits::deg);
    Flipper.stop(brakeType::hold);
}
void storage() {
    Flipper.spin(directionType::fwd, 20, velocityUnits::pct);
}

void lift(){
    LeftLift.spin(directionType::fwd,80,velocityUnits::pct);
    RightLift.spin(directionType::rev,80,velocityUnits::pct);
}
void lift_autonomous() {
    LeftLift.setVelocity(80, velocityUnits::pct);
    RightLift.setVelocity(80, velocityUnits::pct);
    LeftLift.startRotateFor(3000, rotationUnits::deg);
    RightLift.startRotateFor(-3000, rotationUnits::deg);
}
void lower_autonomous() {
    LeftLift.setVelocity(80, velocityUnits::pct);
    RightLift.setVelocity(80, velocityUnits::pct);
    LeftLift.startRotateFor(-3000, rotationUnits::deg);
    RightLift.startRotateFor(3000, rotationUnits::deg);
    vex::task::sleep(2000);
}
void halt() {
    LeftLift.stop(brakeType::hold);
    //RightLift.rotateTo(LeftLift.rotation(rotationUnits::deg), rotationUnits::deg);
    RightLift.stop(brakeType::hold);
}

void lower() {
    LeftLift.spin(directionType::rev,80,velocityUnits::pct);
    RightLift.spin(directionType::fwd,80,velocityUnits::pct);
}
void autonomous( void ) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
    int team = 1;//1 for red, -1 for blue
    spin_up();
    forwardAutonomous(29);
    //turninplaceAutonomous(360);
    turninplaceAutonomous(10);
    shoot_autonomous();
    turninplaceAutonomous(-10);

    unflip_autonomous();
    unflip_autonomous();
    forwardAutonomous(-29);
    forwardAutonomous(-4.25);
    unshoot();

    turninplaceAutonomous(90 * team);
    intake();
    forwardAutonomous(42.75);
    turninplaceAutonomous(-90 * team);
    forwardAutonomous(-15.75);
    flip_autonomous();
    forwardAutonomous(3);

    lift_autonomous();
    turninplaceAutonomous(90 * team);
    forwardAutonomous(-50);

    lower_autonomous();
    unflip_autonomous();
    forwardAutonomous(10);
    forwardAutonomous(-7);
    turninplaceAutonomous(-90 * team);
    forwardAutonomous(68.75);

    turninplaceAutonomous( 45 * team);
    forwardAutonomous(53.335);
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

void usercontrol( void ) {
  // User control code here, inside the loop
  autonomousActive = false;
  while (1){
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
    driveNormal();
    /*if (driveType == 0){
        driveNormal();
    } else {
        driveTank();
    }*/
    Controller2.ButtonR1.pressed(shoot);
    Controller2.ButtonR1.released(unshoot);

    Controller2.ButtonL2.pressed(reverse_intake);
    Controller2.ButtonL2.released(stop_intake);

    Controller2.ButtonL1.pressed(intake);
    Controller2.ButtonL1.released(stop_intake);

    Controller2.ButtonR2.pressed(spin_up);
    Controller2.ButtonR2.released(unspin_up);
    Controller1.ButtonY.pressed(toggleDrive);
    Controller1.ButtonY.released(toggleNull);

    Controller2.ButtonX.pressed(storage);
    Controller2.ButtonX.released(maintain_flip);
    Controller2.ButtonA.pressed(lift);
    Controller2.ButtonA.released(halt);
    Controller2.ButtonB.pressed(lower);
    Controller2.ButtonB.released(halt) ;
    Controller2.ButtonUp.pressed(flip);
    Controller2.ButtonUp.released(maintain_flip);
    Controller2.ButtonDown.pressed(unflip);
    Controller2.ButtonDown.released(maintain_flip);
    vex::task::sleep(20); //Sleep the task for a short amount of time to prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {

    //Run the pre-autonomous function.
    pre_auton();

    //Set up callbacks for autonomous and driver control periods.
    Competition.autonomous( autonomous );
    Competition.drivercontrol( usercontrol );

    //Prevent main from exiting with an infinite loop.
    while(1) {
      vex::task::sleep(100);//Sleep the task for a short amount of time to prevent wasted resources.
    }

}
