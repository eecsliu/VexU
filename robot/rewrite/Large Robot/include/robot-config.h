#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"
//
using namespace vex;
vex::brain Brain;
vex::motor LeftMotorTwo (vex::PORT2, vex::gearSetting::ratio18_1,false);
vex::motor TopIntake (vex::PORT3, vex::gearSetting::ratio18_1,false);
vex::motor FlyWheelMotorOne (vex::PORT4, vex::gearSetting::ratio6_1,false);
vex::motor FlyWheelMotorTwo (vex::PORT8, vex::gearSetting::ratio6_1,false);
vex::motor Flipper (vex::PORT5, vex::gearSetting::ratio36_1,false);
vex::motor AcquisitionMotor (vex::PORT6, vex::gearSetting::ratio18_1,false);
vex::motor BottomIntake (vex::PORT7, vex::gearSetting::ratio18_1,false);
vex::motor LeftLift (vex::PORT9, vex::gearSetting::ratio18_1,false);
vex::motor RightLift (vex::PORT10, vex::gearSetting::ratio18_1,false);
vex::motor RightMotorTwo (vex::PORT11, vex::gearSetting::ratio18_1,false);
vex::motor RightMotorOne (vex::PORT13, vex::gearSetting::ratio18_1,false);
vex::motor LeftMotorOne (vex::PORT15, vex::gearSetting::ratio18_1,false);
vex::bumper LeftSerial( Brain.ThreeWirePort.A);
vex::bumper RightSerial( Brain.ThreeWirePort.B);
vex::bumper Ready( Brain.ThreeWirePort.D);
vex::bumper Running( Brain.ThreeWirePort.C);
vex::controller Controller1;
vex::controller Controller2( vex::controllerType::partner );