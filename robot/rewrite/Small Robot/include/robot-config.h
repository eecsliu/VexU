#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"
//
using namespace vex;
vex::brain Brain;
vex::motor TopIntake (vex::PORT2, vex::gearSetting::ratio18_1,true);
vex::motor BottomIntake (vex::PORT4, vex::gearSetting::ratio18_1,true);
vex::motor FlyWheelMotorOne (vex::PORT6, vex::gearSetting::ratio6_1,false);
vex::motor FlyWheelMotorTwo (vex::PORT7, vex::gearSetting::ratio6_1,false);
vex::motor LeftMotorOne (vex::PORT11, vex::gearSetting::ratio18_1,false);
vex::motor RightMotorOne (vex::PORT12, vex::gearSetting::ratio18_1,false);
vex::motor LeftMotorTwo (vex::PORT13, vex::gearSetting::ratio18_1,false);
vex::motor RightMotorTwo (vex::PORT14, vex::gearSetting::ratio18_1,false);
vex::motor LiftOne (vex::PORT3, vex::gearSetting::ratio18_1, false);
vex::motor LiftTwo (vex::PORT5, vex::gearSetting::ratio18_1, false);
vex::motor Hammer (vex::PORT9, vex::gearSetting::ratio36_1,false);
vex::bumper LeftAnalog (Brain.ThreeWirePort.A);
vex::bumper RightAnalog (Brain.ThreeWirePort.B);
vex::controller Controller1;