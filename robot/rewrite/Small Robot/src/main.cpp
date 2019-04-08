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
    LeftMotor.setTimeout(10, timeUnits::sec);
    LeftMotor2.setTimeout(10, timeUnits::sec);
    RightMotor.setTimeout(10, timeUnits::sec);
    RightMotor2.setTimeout(10, timeUnits::sec);
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
const double robotWidthPos = ((11.25 * (360.0 / 339.56)) * (357.65 / 360))* ((3600 + 9.8742689841187) / 3600) * (3605.527/3600.0);
bool driveType = false;
void forwardAutonomous(double distance, double speed) {
    autonomousActive = true;
    LeftMotor.setVelocity(speed, velocityUnits::pct);
    LeftMotor2.setVelocity(speed, velocityUnits::pct);
    RightMotor.setVelocity(speed, velocityUnits::pct);
    RightMotor2.setVelocity(speed, velocityUnits::pct);
    //Converts linear distance to rotations of the wheel using formula
    //Then, rotates wheels accordingly
    LeftMotor.startRotateFor(360 * distance / wheelCircumference, rotationUnits::deg);
    LeftMotor2.startRotateFor(360 * distance / wheelCircumference, rotationUnits::deg);
    RightMotor.startRotateFor(-360 * distance / wheelCircumference, rotationUnits::deg);
    RightMotor2.rotateFor(-360 * distance / wheelCircumference, rotationUnits::deg);
    autonomousActive = false;
}

void turninplaceAutonomous(double degrees) {
    autonomousActive = true;
    LeftMotor.setVelocity(30, velocityUnits::pct);
    LeftMotor2.setVelocity(30, velocityUnits::pct);
    RightMotor.setVelocity(30, velocityUnits::pct);
    RightMotor2.setVelocity(30, velocityUnits::pct);
    double robotWidth = robotWidthPos;
    if(degrees < 0.0) {
        robotWidth = robotWidth * (3600 + 9.27)/3600 * (3600 - 3.7)/3600;
        Controller1.Screen.print("Neg");
    }
    else {
        Controller1.Screen.print("Pos");
    }
    //Creates an imaginary circle that is the circle of rotation
    //that the robot would rotate in. Calculates the circumference of that
    //circle and rotates the motors that number of degrees
    LeftMotor.startRotateFor(degrees * robotWidth / wheelDiameter, rotationUnits::deg);
    LeftMotor2.startRotateFor(degrees * robotWidth / wheelDiameter, rotationUnits::deg);
    RightMotor.startRotateFor(degrees * robotWidth / wheelDiameter, rotationUnits::deg);
    RightMotor2.rotateFor(degrees * robotWidth / wheelDiameter, rotationUnits::deg);
    autonomousActive = false;
}

void climb() {
    LeftMotor.setVelocity(100, velocityUnits::pct);
    RightMotor.setVelocity(100, velocityUnits::pct);
    LeftMotor2.setVelocity(100, velocityUnits::pct);
    RightMotor2.setVelocity(100, velocityUnits::pct);
    //Drives forward 45 inches at 50 % speed
    LeftMotor.spin(directionType::rev);
    LeftMotor2.spin(directionType::rev);
    RightMotor.spin(directionType::fwd);
    RightMotor2.spin(directionType::fwd);
    Hammer.setVelocity(90, velocityUnits::pct);
    Hammer.rotateTo(-800, rotationUnits::deg);
    vex::task::sleep(300);
    Hammer.rotateTo(-500, rotationUnits::deg);
    Hammer.rotateTo(-800, rotationUnits::deg);
    vex::task::sleep(860);
    LeftMotor.stop(brakeType::hold);
    LeftMotor2.stop(brakeType::hold);
    RightMotor.stop(brakeType::hold);
    RightMotor2.stop(brakeType::hold);
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
        LeftMotor.spin(directionType::rev, - leftPwm, velocityUnits::pct);
        LeftMotor2.spin(directionType::rev, - leftPwm, velocityUnits::pct);
    }
    else{
        LeftMotor.spin(directionType::fwd, leftPwm, velocityUnits::pct);
        LeftMotor2.spin(directionType::fwd, leftPwm, velocityUnits::pct);
    }
    if (rightPwm < 0){
        RightMotor.spin(directionType::fwd, -rightPwm, velocityUnits::pct);
        RightMotor2.spin(directionType::fwd, -rightPwm, velocityUnits::pct);

    }
    else{
        RightMotor.spin(directionType::rev, rightPwm, velocityUnits::pct);
        RightMotor2.spin(directionType::rev, rightPwm, velocityUnits::pct);
    }
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
    stop_intake();
}
void place_ball() {
    TopIntake.spin(directionType::rev, 100, velocityUnits::pct);
}
void place_ball_control() {
    if (Controller1.ButtonR2.pressing()) {
        TopIntake.spin(directionType::rev, 100, velocityUnits::pct);
    }
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
    //calculatePower(getDistance());
    FlyWheelMotor.spin(directionType::rev, 100, velocityUnits::pct);
    intake();
}
void shoot_autonomous() {
    //calculatePower(getDistance());
    FlyWheelMotor.spin(directionType::rev, 100, velocityUnits::pct);
    //vex::task::sleep(1000);
    place_ball();
    vex::task::sleep(2000);
}

double max_rotation = 180;
/*
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
*/
void unjamShooter() {
    TopIntake.spin(directionType::fwd, 50, velocityUnits::pct);
}
void stopUnjam() {
    TopIntake.stop();
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
    /*int team = -1; //1 for red, -1 for blue
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
    robot_skills();
    //turninplaceAutonomous(-90);
    //turninplaceAutonomous(3600);
    //climb();
    //turninplaceAutonomous(1800);
    //vex::task::sleep(3000);
    //turninplaceAutonomous(-1800);
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
     /*if (Controller1.ButtonR2.pressing()){
        if (FlyWheelMotor.velocity(velocityUnits::pct) < -90) {
              place_ball();
        }
      }*/
      Controller1.ButtonR1.pressed(place_ball_control);

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

    if (Controller1.ButtonX.pressing()) {
        Hammer.spin(directionType::fwd, 80, velocityUnits::pct);
    } else if (Controller1.ButtonB.pressing()) {
        Hammer.spin(directionType::rev, 80, velocityUnits::pct);
    } else {
        Hammer.stop(brakeType::hold);
    }
    /*
    Controller2.ButtonX.pressed(storage);
    Controller2.ButtonX.released(maintain_flip);
    Controller2.ButtonA.pressed(lift);
    Controller2.ButtonA.released(halt);
    Controller1.ButtonB.pressed(toggleDrive);
    Controller1.ButtonB.released(toggleNull) ;
    Controller1.ButtonUp.pressed(flip);
    Controller1.ButtonUp.released(maintain_flip);
    Controller1.ButtonDown.pressed(unflip);
    Controller1.ButtonDown.released(maintain_flip);
    vex::task::sleep(20); //Sleep the task for a short amount of time to prevent wasted resources.
    */
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
