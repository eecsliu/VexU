#include "robot-config.h"
#define _USE_MATH_DEFINES
using namespace std;
#include <list>
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
  //flipperStartPosition = Flipper.rotation(rotationUnits::deg)-180;
    LeftMotorOne.setTimeout(10, timeUnits::sec);
    LeftMotorTwo.setTimeout(10, timeUnits::sec);
    RightMotorOne.setTimeout(10, timeUnits::sec);
    RightMotorTwo.setTimeout(10, timeUnits::sec);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                           * ((3600 - 1.06) / 3600         */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
bool autonomousActive = false;
const double wheelDiameter = 4;
const double wheelCircumference = wheelDiameter * M_PI;
const double robotWidthPos = 11.9000847212;
const double convertDegrees = (360) / (2 * M_PI);
const double convertRadians = 1 / convertDegrees;
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
  orientation = fmod(orientation, 360);
}
void goTo(double x, double y) {
  double xDelta = x - xPosition;
  double yDelta = y - yPosition;
  double side = 0;
  if (yDelta < 0) {
    side = 180;
  }
  rotateTo(atan2(xDelta, yDelta) * convertDegrees);
  forwardAutonomous(sqrt(xDelta * xDelta + (yDelta * yDelta)));
}
void goTo(double x, double y, double degrees) {
  goTo(x, y);
  rotateTo(degrees);
}
void climbLow() {
  LeftMotorOne.setVelocity(100, velocityUnits::pct);
  RightMotorOne.setVelocity(100, velocityUnits::pct);
  LeftMotorTwo.setVelocity(100, velocityUnits::pct);
  RightMotorTwo.setVelocity(100, velocityUnits::pct);
  //Drives forward 45 inches at 50 % speed
  LeftMotorOne.spin(directionType::rev);
  LeftMotorTwo.spin(directionType::rev);
  RightMotorOne.spin(directionType::fwd);
  RightMotorTwo.spin(directionType::fwd);
  Hammer.setVelocity(90, velocityUnits::pct);
  Hammer.rotateTo(-800, rotationUnits::deg);
  vex::task::sleep(400);
  LeftMotorOne.stop(brakeType::hold);
  LeftMotorTwo.stop(brakeType::hold);
  RightMotorOne.stop(brakeType::hold);
  RightMotorTwo.stop(brakeType::hold);
}
void climb() {
    LeftMotorOne.setVelocity(100, velocityUnits::pct);
    RightMotorOne.setVelocity(100, velocityUnits::pct);
    LeftMotorTwo.setVelocity(100, velocityUnits::pct);
    RightMotorTwo.setVelocity(100, velocityUnits::pct);
    //Drives forward 45 inches at 50 % speed
    LeftMotorOne.spin(directionType::rev);
    LeftMotorTwo.spin(directionType::rev);
    RightMotorOne.spin(directionType::fwd);
    RightMotorTwo.spin(directionType::fwd);
    Hammer.setVelocity(90, velocityUnits::pct);
    Hammer.rotateTo(-800, rotationUnits::deg);
    vex::task::sleep(300);
    Hammer.rotateTo(-500, rotationUnits::deg);
    Hammer.rotateTo(-800, rotationUnits::deg);
    vex::task::sleep(860);
    LeftMotorOne.stop(brakeType::hold);
    LeftMotorTwo.stop(brakeType::hold);
    RightMotorOne.stop(brakeType::hold);
    RightMotorTwo.stop(brakeType::hold);
}

void activate_motors(double angularPower, double linearPower){
    //When the motors are low power, just convert to 10.
    //This creates a dead zone and prevents drifting
    if (abs(linearPower) < 10) {
        linearPower = 0;
    }
    if (abs(angularPower) < 10) {
        angularPower = 0;
    }
    //Sets both motors to the linear power. Then, adjusts
    //speed of left and right motors based off angular amount
    double rightPwm = linearPower;
    double leftPwm = linearPower;
    leftPwm -= angularPower;
    rightPwm += angularPower;
    //If power is greater than 100% for a side, just set to 100% and change
    //other side accordingly
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
    //Spins the motors the right direction at the calculated power
    if (leftPwm < 0){
        LeftMotorOne.spin(directionType::rev, - leftPwm, velocityUnits::pct);
        LeftMotorTwo.spin(directionType::rev, - leftPwm, velocityUnits::pct);
    }
    else{
        LeftMotorOne.spin(directionType::fwd, leftPwm, velocityUnits::pct);
        LeftMotorTwo.spin(directionType::fwd, leftPwm, velocityUnits::pct);
    }
    if (rightPwm < 0){
        RightMotorOne.spin(directionType::fwd, -rightPwm, velocityUnits::pct);
        RightMotorTwo.spin(directionType::fwd, -rightPwm, velocityUnits::pct);

    }
    else{
        RightMotorOne.spin(directionType::rev, rightPwm, velocityUnits::pct);
        RightMotorTwo.spin(directionType::rev, rightPwm, velocityUnits::pct);
    }
}

void stopAim() {
  LeftMotorOne.stop();
  LeftMotorTwo.stop();
  RightMotorOne.stop();
  RightMotorTwo.stop();
}

bool aimShooter(){
  int serialInstruction = LeftAnalog.value() - RightAnalog.value();
  Brain.Screen.clearLine();
  Brain.Screen.print(serialInstruction);
  int adjustmentSpeed = 8;
  if (serialInstruction > 0) {
    LeftMotorOne.spin(directionType::rev, adjustmentSpeed, velocityUnits::pct);
    LeftMotorTwo.spin(directionType::rev, adjustmentSpeed, velocityUnits::pct);
    RightMotorOne.spin(directionType::rev, adjustmentSpeed, velocityUnits::pct);
    RightMotorTwo.spin(directionType::rev, adjustmentSpeed, velocityUnits::pct);
    return true;
  } else if (serialInstruction < 0) {
    LeftMotorOne.spin(directionType::fwd, adjustmentSpeed, velocityUnits::pct);
    LeftMotorTwo.spin(directionType::fwd, adjustmentSpeed, velocityUnits::pct);
    RightMotorOne.spin(directionType::fwd, adjustmentSpeed, velocityUnits::pct);
    RightMotorTwo.spin(directionType::fwd, adjustmentSpeed, velocityUnits::pct);
    return true;
  }
  stopAim();
  return false;
}

void driveNormal(){
    if (autonomousActive == false){
        double SENSITIVITY_CONSTANT;
        //When r1 is pressed, turns become slower so it's easier to aim
        if (Controller1.ButtonR1.pressing()) {
            SENSITIVITY_CONSTANT = -0.15;
        }
        else {
            SENSITIVITY_CONSTANT = -0.30;
        }
        //Calculate forward power based off direction
        double linearPower = Controller1.Axis3.position(percentUnits::pct) * direction;
        if (Controller1.ButtonUp.pressing()) {
            linearPower = 40;
        } else if (Controller1.ButtonDown.pressing()) {
            linearPower = -40;
        }
        //Multiply angular power by SENSITIVITY_CONSTANT so that it's possible
        //to change how fast robot turns
        double angularPower = Controller1.Axis1.value() * SENSITIVITY_CONSTANT;
        activate_motors(angularPower, linearPower);
    }
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
void driveTank(){
    if (autonomousActive == false) {
        LeftMotorOne.spin(directionType::fwd, Controller1.Axis3.position(percentUnits::pct), velocityUnits::pct);
        LeftMotorTwo.spin(directionType::fwd, Controller1.Axis3.position(percentUnits::pct), velocityUnits::pct);
        RightMotorOne.spin(directionType::rev, Controller1.Axis2.position(percentUnits::pct), velocityUnits::pct);
        RightMotorTwo.spin(directionType::rev, Controller1.Axis2.position(percentUnits::pct), velocityUnits::pct);
    }
}
void toggleDrive(){
    direction = -direction;
    
}

void toggleNull() {
    return;
}

void unshoot() {
    FlyWheelMotorOne.stop();
    FlyWheelMotorTwo.stop();
    TopIntake.stop();
    stop_intake();
}
void place_ball() {
    TopIntake.spin(directionType::rev, 100, velocityUnits::pct);
}
void place_one_ball() {
    TopIntake.startRotateFor(directionType::rev, 800, rotationUnits::deg, 100, velocityUnits::pct);
}
void place_ball_control() {
    if (Controller1.ButtonR2.pressing()) {
        TopIntake.spin(directionType::rev, 100, velocityUnits::pct);
    }
}
void stop_place() {
  TopIntake.stop();
}
void place_ball_autonomous() {
    TopIntake.spin(directionType::rev, 100, velocityUnits::pct);
    while (TopIntake.velocity(velocityUnits::pct) < 95.0) {
        continue;
    }
    unshoot();
}
void spin_up() {
    FlyWheelMotorOne.spin(directionType::rev, 600, velocityUnits::rpm);
    FlyWheelMotorTwo.spin(directionType::rev, 600, velocityUnits::rpm);
}
void unspin_up() {
    FlyWheelMotorOne.stop();
    FlyWheelMotorTwo.stop();
}

void shoot(){
    //calculatePower(getDistance());
    //calculatePower(getDistance());
    FlyWheelMotorOne.spin(directionType::rev, 100, velocityUnits::pct);
    FlyWheelMotorTwo.spin(directionType::rev, 100, velocityUnits::pct);
    intake();
}
void shoot_autonomous() {
    //calculatePower(getDistance());
    Brain.resetTimer();
    LeftMotorOne.resetRotation();
    LeftMotorTwo.resetRotation();
    RightMotorOne.resetRotation();
    RightMotorTwo.resetRotation();
    while (Brain.timer(timeUnits::sec) < 2){
      aimShooter();
    }
    double rotationChange = (LeftMotorOne.rotation(rotationUnits::deg) + LeftMotorTwo.rotation(rotationUnits::deg) + RightMotorOne.rotation(rotationUnits::deg) + RightMotorTwo.rotation(rotationUnits::deg))/4;
    double degreeChange = rotationChange * wheelDiameter / (robotWidthPos);
    stopAim();
    FlyWheelMotorOne.spin(directionType::rev, 100, velocityUnits::pct);
    FlyWheelMotorTwo.spin(directionType::rev, 100, velocityUnits::pct);
    //vex::task::sleep(1000);
    place_ball();
    vex::task::sleep(2000);
    turninplaceAutonomous(-degreeChange);
    unshoot();
}

void unjamShooter() {
    TopIntake.spin(directionType::fwd, 50, velocityUnits::pct);
}
void stopUnjam() {
    TopIntake.stop();
}
void stopLift() {
  LiftOne.stop(brakeType::hold);
  LiftTwo.stop(brakeType::hold);
}
void lift() {
  LiftOne.spin(directionType::fwd, 20, velocityUnits::pct);
  LiftTwo.spin(directionType::rev, 20, velocityUnits::pct);
}
void lower() {
  LiftOne.spin(directionType::rev, 20, velocityUnits::pct);
  LiftTwo.spin(directionType::fwd, 20, velocityUnits::pct);
}
void robot_skills() {
    /*
    spin_up();
    forwardAutonomous(23.75);
    turninplaceAutonomous(10);
    shoot_autonomous();
    turninplaceAutonomous(-10);
    forwardAutonomous(21);
    unshoot();
    forwardAutonomous(-(23.75 + 21 + 4.5));
    turninplaceAutonomous(90);
    intake();
    forwardAutonomous(47);
    turninplaceAutonomous(-90);
    spin_up();
    forwardAutonomous(4.5 + 23.75);
    turninplaceAutonomous(10);
    shoot_autonomous();
    turninplaceAutonomous(-10);
    forwardAutonomous(21);
    unshoot();
    intake();
    forwardAutonomous(-21);
    turninplaceAutonomous(90);
    forwardAutonomous(23.75 * 1.5);
    turninplaceAutonomous(90);
    forwardAutonomous(23.75 + 6.5);
    turninplaceAutonomous(90);
    forwardAutonomous(10);
    forwardAutonomous(-10);
    turninplaceAutonomous(90);
    forwardAutonomous(23.75);
    turninplaceAutonomous(90);
    forwardAutonomous((23.75 * .5)+4);
    turninplaceAutonomous(-90);
    spin_up();
    forwardAutonomous(6.5);
    turninplaceAutonomous(10);
    shoot_autonomous();
    turninplaceAutonomous(-10);
    forwardAutonomous(21);
    unshoot();
    intake();
    forwardAutonomous(-(21+4.5+11.5));
    turninplaceAutonomous(-90);
    forwardAutonomous(2.5 * 23.75 + 4);
    turninplaceAutonomous(-90);
    climb();*/
    int default_speed = 33;
    Hammer.stop(brakeType::hold);
    spin_up();
    forwardAutonomous(23.75, default_speed);
    //turninplaceAutonomous(10);
    shoot_autonomous();
    //turninplaceAutonomous(-10);
    unshoot();
    forwardAutonomous(20.5, default_speed);
    forwardAutonomous(-(20.5 + 5), default_speed);
    turninplaceAutonomous(91);
    reverse_intake();
    forwardAutonomous(23.75, default_speed);
    turninplaceAutonomous(91);
    forwardAutonomous(23.75 + .5, 40);
    turninplaceAutonomous(-90);
    reverse_intake();
    forwardAutonomous(23.75*1, 40);
    stop_intake();
    intake();
    vex::task::sleep(1000);
    spin_up();
    forwardAutonomous(-1, default_speed);
    turninplaceAutonomous(-90);
    forwardAutonomous(23.75 + 5.5, default_speed);
    turninplaceAutonomous(10);
    stop_intake();
    shoot_autonomous();
    turninplaceAutonomous(-10);
    unshoot();
    //forwardAutonomous(21, default_speed);
    //forwardAutonomous(-(21 + 5.5 + 1), default_speed);//backing away from flag
    forwardAutonomous(-(5.5 + 1), default_speed);
    turninplaceAutonomous(91);
    forwardAutonomous(23.75 + 1, default_speed);
    reverse_intake();//last cap
    forwardAutonomous(23.75, 40);
    stop_intake();
    turninplaceAutonomous(90);
    forwardAutonomous(23.75 + 1, default_speed);
    turninplaceAutonomous(91);
    intake();
    forwardAutonomous(26.75, 40);
    //stop_intake();
    //intake();
    //vex::task::sleep(1000);
    spin_up();
    forwardAutonomous(-26.75, default_speed);
    stop_intake();
    turninplaceAutonomous(91);
    forwardAutonomous(23.75 + 5.5 - 1, default_speed);
    stop_intake();
    turninplaceAutonomous(10);
    shoot_autonomous();
    turninplaceAutonomous(-5);
    unshoot();
    forwardAutonomous(18.5, default_speed);
    forwardAutonomous(-(20.5 + 5.5 + 23.75), default_speed);
    turninplaceAutonomous(-90);
    forwardAutonomous(-10, default_speed);
    turninplaceAutonomous(90);
    forwardAutonomous(-(23.75 - 2), default_speed);
    turninplaceAutonomous(90);
    forwardAutonomous(-1, default_speed);
    climb();
    /*forwardAutonomous(23.75, 30);

    turninplaceAutonomous(10);
    shoot_autonomous();
     turninplaceAutonomous(-10);
    unshoot();
    forwardAutonomous(21, 30);
    //unshoot();
    forwardAutonomous(-(21 + 5.5), 30);
    turninplaceAutonomous(90);
    reverse_intake();
    forwardAutonomous(23.75, 30);
    //stop_intake();
    turninplaceAutonomous(90);
    forwardAutonomous(23.75, 30);
    turninplaceAutonomous(-90);
    //reverse_intake();
    forwardAutonomous(23.75 * .9, 50);//Running into first sideways boi
    //stop_intake();
    turninplaceAutonomous(-90);
    forwardAutonomous(23.75, 30);
    turninplaceAutonomous(90);
    //forwardAutonomous(23.75*.2, 30);
    turninplaceAutonomous(-90);
    forwardAutonomous(21, 30);
    forwardAutonomous(-21, 30);
    turninplaceAutonomous(90);
    //reverse_intake();
    forwardAutonomous(23.75*2, 30);
    //stop_intake();
    turninplaceAutonomous(-90);
    forwardAutonomous(21, 30);
    forwardAutonomous(-(21 + 23.75), 30);
    turninplaceAutonomous(-90);
    //reverse_intake();
    forwardAutonomous(23.75*1.75, 30);
    //stop_intake();
    forwardAutonomous(-23.75*1.75, 30);
    turninplaceAutonomous(-90);
    forwardAutonomous(2*23.75, 30);
    turninplaceAutonomous(90);
    //reverse_intake();
    forwardAutonomous(23.75*1.75, 30);
    //stop_intake();
    forwardAutonomous(-23.75*1.75, 30);
    turninplaceAutonomous(-90);
    forwardAutonomous(23.75, 30);
    turninplaceAutonomous(90);
    //reverse_intake();
    forwardAutonomous(23.75*1.75, 30);
    //stop_intake();
    forwardAutonomous(-23.75*1.75, 30);
    turninplaceAutonomous(90);
    forwardAutonomous(23.75*.5, 30);
    turninplaceAutonomous(-90);
    forwardAutonomous(23.75*3, 30);
    turninplaceAutonomous(-90);
    forwardAutonomous(23.75*.5, 30);
    turninplaceAutonomous(-90);
    //reverse_intake();
    forwardAutonomous(23.75*1.75, 30);
    //stop_intake();
    forwardAutonomous(-23.75*1.75, 30);
    turninplaceAutonomous(-90);
    forwardAutonomous(23.75, 30);
    turninplaceAutonomous(90);
    //reverse_intake();
    forwardAutonomous(23.75*1.75, 30);
    stop_intake();
    forwardAutonomous(-23.75*.75, 30);
    turninplaceAutonomous(-90);
    climb();
    turninplaceAutonomous(90);
    climb();*/
}

void autonomous( void ) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
    int team = -1; //1 for red, -1 for blue
    const double TILE_WIDTH = 23.75;
    spin_up();
    goTo(0, TILE_WIDTH);
    shoot_autonomous();
    unshoot();
    
    goTo(0, TILE_WIDTH * 1.9);
    forwardAutonomous(-(TILE_WIDTH * 1.9 + 5));
    turninplaceAutonomous(90 * team);
    intake();
    forwardAutonomous(TILE_WIDTH * 2);
    forwardAutonomous(-2);
    turninplaceAutonomous(-90 * team);
    goTo(1.95 * TILE_WIDTH * team, 1.9 * TILE_WIDTH, 0);
    forwardAutonomous(-20);
    spin_up();
    goTo(team * TILE_WIDTH * 1.35, TILE_WIDTH * 1.25, 45 * team);
    shoot_autonomous();

    forwardAutonomous(7);
    turninplaceAutonomous(team * -45);
    forwardAutonomous(-1.25 * TILE_WIDTH);
    climbLow();

    /*
    spin_up();
    forwardAutonomous(23.5);
    turninplaceAutonomous(10 * team);
    shoot_autonomous();
    turninplaceAutonomous(-10 * team);
    forwardAutonomous(21);
    unshoot();
    forwardAutonomous(-(23.5 + 21 + 4.5));
    turninplaceAutonomous(90 * team);
    intake();
    forwardAutonomous(47);
    turninplaceAutonomous(-90 * team);
    forwardAutonomous(23.5 + 21 + 4.5);
    forwardAutonomous(-12);
    turninplaceAutonomous(-90 * team);
    forwardAutonomous(12);
    turninplaceAutonomous((90 + 45) * team);
    stop_intake();
    shoot_autonomous();
    turninplaceAutonomous((90 + 45) * team);
    unshoot();
    forwardAutonomous(25);
    climb();*/
    //shoot_autonomous();
    //robot_skills();
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
int counter = 0;
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
    
    
    Controller1.Screen.print(-FlyWheelMotorTwo.velocity(velocityUnits::rpm));
    driveNormal();
    //Controller1.ButtonR1.pressed(place_ball);
    Controller1.ButtonR1.released(place_one_ball);

    Controller1.ButtonL2.pressed(reverse_intake);
    Controller1.ButtonL2.released(stop_intake);

    Controller1.ButtonUp.pressed(reverse_intake);
    Controller1.ButtonUp.released(stop_intake);

    Controller1.ButtonL1.pressed(intake);
    Controller1.ButtonL1.released(stop_intake);

    Controller1.ButtonR2.pressed(shoot);
    Controller1.ButtonR2.released(unshoot);

    Controller1.ButtonA.pressed(unjamShooter);
    Controller1.ButtonA.released(stopUnjam);

    Controller1.ButtonLeft.pressed(lower);
    Controller1.ButtonLeft.released(stopLift);
    Controller1.ButtonRight.pressed(lift);
    Controller1.ButtonRight.released(stopLift);

    if (Controller1.ButtonY.pressing()) {
      aimShooter();
    }
    Controller1.ButtonY.released(stopAim);
    if (Controller1.ButtonX.pressing()) {
      Hammer.spin(directionType::fwd, 100, velocityUnits::pct);
    } else if (Controller1.ButtonB.pressing()) {
      Hammer.spin(directionType::rev, 100, velocityUnits::pct);
    } else {
      Hammer.stop(brakeType::hold);
    }
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
