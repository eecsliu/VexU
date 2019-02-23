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

void pre_auton( void ) {
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...

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
    BottomIntake.stop();
}


bool autonomousActive = false;
const double wheelDiameter = 4;
const double wheelCircumference = wheelDiameter * M_PI;
const double robotWidth = 11.75;
bool driveType = false;
void forwardAutonomous(double distance) {
    autonomousActive = true;
    LeftMotor.setVelocity(25, velocityUnits::pct);
    RightMotor.setVelocity(25, velocityUnits::pct);
    LeftMotor.startRotateFor(360 * distance / wheelCircumference, rotationUnits::deg);
    RightMotor.rotateFor(-360 * distance / wheelCircumference, rotationUnits::deg);
    autonomousActive = false;
    vex::task::sleep(300);
    //vexGenericSerialEnable( vex::PORT18, 0 );
    //https://www.vexforum.com/index.php/34239-vex-u?search=serial
}

void shoot_autonomous() {
    FlyWheelMotor.spin(directionType::rev, 10, velocityUnits::pct);
    BottomIntake.spin(directionType::fwd, 10, velocityUnits::pct);
    vex::task::sleep(200);
    FlyWheelMotor.stop();
    BottomIntake.stop();
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
    LeftMotor.setVelocity(25, velocityUnits::pct);
    RightMotor.setVelocity(25, velocityUnits::pct);
    LeftMotor.startRotateFor(degrees * robotWidth / wheelDiameter, rotationUnits::deg);
    RightMotor.rotateFor(degrees * robotWidth / wheelDiameter, rotationUnits::deg);
    autonomousActive = false;
}
void forwardTest(){
    forwardAutonomous(200);
}
void backwardTest(){
    forwardAutonomous(-200);
}
void leftTest(){
    turninplaceAutonomous(-360);
}
void rightTest(){
    turninplaceAutonomous(360);
}

void autonomous( void ) {
    forwardAutonomous(8);
    shoot_autonomous();
    forwardAutonomous(35);
    forwardAutonomous(-15);
    turninplaceAutonomous(-90);
    forwardAutonomous(32);
    turninplaceAutonomous(90);
    forwardAutonomous(15);
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................

}
// int receiveTask() {
//     // enable port 18 as generic serial port
//     vexGenericSerialEnable( vex::PORT12, 0 );
//     // change baud rate, default is 230k
//     vexGenericSerialBaudrate( vex::PORT12, 115200 );
//     // allow vexos to reconfigure the port
//     // the port will remain as a generic serial port until the brain is power cycled
//     this_thread::sleep_for(10);

//     // buffer for rx data
//     uint8_t rxbuf[256];

//     int totalRead = 0;

//     while(1) {
//       // check to see if we have any bytes in the receive buffer
//       int nRead = vexGenericSerialReceive( vex::PORT12, rxbuf, sizeof(rxbuf)  );

//       // yes ? then display
//       if( nRead > 0 ) {
//         totalRead += nRead;
//         Brain.Screen.setCursor( 4, 2 );
//         Brain.Screen.print("Bytes received %d", totalRead );


//         if( totalRead > 16 )
//           totalRead = 16;
//         Brain.Screen.setCursor( 5, 2 );
//         Brain.Screen.clearLine();
//         Brain.Screen.setCursor( 5, 2 );
//         for(int i=0;i<nRead;i++)
//           Brain.Screen.print("%02X ", rxbuf[i] );
//       }

//       // read often
//     }

//     return(0);
// }
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
        double SENSITIVITY_CONSTANT = -0.25;
        double linearPower = Controller1.Axis3.position(percentUnits::pct);
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
    driveType = !driveType;
    Controller1.Screen.print(driveType);
}

void toggleNull() {
    return;
}

list<motor> lift_motors;
//TODO Add lift motors
//lift_motors.push_back();
const int LOW_HEIGHT = 0;
const int HIGH_HEIGHT = 1;
void liftArm(int height){
    if (height == LOW_HEIGHT){
        LiftMotor.startRotateTo(90, rotationUnits::deg);
    }
    else{
        LiftMotor.startRotateTo(0, rotationUnits::deg);
    }
}
void liftHigh(){
    liftArm(HIGH_HEIGHT);
}
void liftLow(){
    liftArm(LOW_HEIGHT);
}
void resetArm(){
    //liftHelperFunction(reset_switch, directionType::rev);
}
/*void liftHelperFunction(switch, direction){
    while(not switch.is_pressed()){
        for (int i = 0; i < lift_motors.size(); i ++){
            lift_motors[i].spin(direction, 50);
        }
    }
}*/
const int FAST_SPEED = 100;
const int SLOW_SPEED = 50;
void toggle_slow(){
    if (BottomIntake.isSpinning() == 0){
        BottomIntake.spin(directionType::fwd, SLOW_SPEED, velocityUnits::pct);
    }
    else{
        BottomIntake.stop();
    }
}
void toggle_fast(){
    if (BottomIntake.isSpinning() == 0){
        BottomIntake.spin(directionType::fwd, FAST_SPEED, velocityUnits::pct);
    }
    else{
        BottomIntake.stop();
    }
}

void toggle_slow_back(){
    if (BottomIntake.isSpinning() == 0){
        BottomIntake.spin(directionType::rev, SLOW_SPEED, velocityUnits::pct);
    }
    else{
        BottomIntake.stop();
    }
}

void toggle_fast_back(){
    if (BottomIntake.isSpinning() == 0){
        BottomIntake.spin(directionType::rev, FAST_SPEED, velocityUnits::pct);
    }
    else{
        BottomIntake.stop();
    }
}

double startPositionAcquisition = AcquisitionMotor.rotation(rotationUnits::deg);
void toggleAcquisitionHelper (double degrees){
    AcquisitionMotor.startRotateTo(startPositionAcquisition + degrees, rotationUnits::deg);
}
void toggleAcquisitionStowed(){
    while (!(StowedLimit.pressing())){
        AcquisitionMotor.spin(directionType::rev, 50, velocityUnits::pct);
    }
}
void toggleAcquisitionFlipped(){
    AcquisitionMotor.startRotateTo(startPositionAcquisition + 180, rotationUnits::deg);
}
void toggleAcquisitionGrabbing(){
    while (!(GrabbingLimit.pressing())){
        AcquisitionMotor.spin(directionType::fwd, 100, velocityUnits::pct);
    }
}
/*
double getDistance(){
    return 10;
}
double calculatePower(double distance){
    FlyWheelMotor.spin(directionType::fwd, distance, velocityUnits::rpm);
}
double getOrientationChange(){
    return 10;
}
void aim(){
    turninplaceAutonomous(getOrientationChange());
}
void place_ball(){

}*/
void shoot(){
    //calculatePower(getDistance());
    //place_ball();
    FlyWheelMotor.spin(directionType::rev, 10, velocityUnits::pct);
    BottomIntake.spin(directionType::fwd, 10, velocityUnits::pct);
    vex::task::sleep(200);
    FlyWheelMotor.stop();
    BottomIntake.stop();
    if (!Controller1.ButtonR1.pressing()) {
        return;
    }
    FlyWheelMotor.spin(directionType::fwd, 100, velocityUnits::pct);
    while (FlyWheelMotor.velocity(velocityUnits::pct) < 95 && Controller1.ButtonR1.pressing()) {
        continue;
    }
    if (FlyWheelMotor.velocity(velocityUnits::pct) >= 95 && Controller1.ButtonR1.pressing()){
        BottomIntake.spin(directionType::rev, 50, velocityUnits::pct);
    }
}


void intake(){
    BottomIntake.spin(directionType::rev, 50, velocityUnits::pct);
}

void stop_intake() {
    BottomIntake.stop();
}

        /*
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0()
Vector3 accelZero
Vector3 magZero
Vector3 gyroZero
float orientation

//Runs in the setup
def zero():
    lsm.begin()
    lsm.read()
    accelZero = lsm.accelData
    magZero = lsm.magData
    gyroZero = lsm.gyroData

//Runs in the loop
def update_orientation():
    lsm.read()
    accelOffset = lsm.accelData - accelZero;
    magOffset = lsm.magData - magZero;
    gyroOffset = lsm.gyroData - gyroZero;
    orientation = (magOffset.x +gyroOffset.y) / 2;
*/
void usercontrol( void ) {
  // User control code here, inside the loop
  while (1){
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
    //This code is for drive train
    if (driveType == 0){
        driveNormal();
    } else {
        driveTank();
    }
    //This code is for the initial intake
    //Controller1.ButtonA.pressed(toggle_slow);
    //Controller1.ButtonB.pressed(toggle_slow_back);
    //Controller1.ButtonX.pressed(toggle_fast);
    //Controller1.ButtonY.pressed(toggle_fast_back);
    //Controller1.ButtonUp.pressed(liftHigh);
      Controller1.ButtonR1.pressed(shoot);
      Controller1.ButtonR1.released(unshoot);
      Controller1.ButtonR2.pressed(intake);
      Controller1.ButtonR2.released(stop_intake);
      Controller1.ButtonA.pressed(toggleDrive);
      Controller1.ButtonA.released(toggleNull);
    //Controller1.ButtonDown.pressed(liftLow);
    //Controller1.ButtonLeft.pressed(leftTest);
    //Controller1.ButtonRight.pressed(rightTest);
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
    // while(1) {
    //   vex::task::sleep(100);//Sleep the task for a short amount of time to prevent wasted resources.
    // }

}
