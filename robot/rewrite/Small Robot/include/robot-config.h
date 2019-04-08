#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"
//
using namespace vex;
vex::brain Brain;
vex::motor TopIntake (vex::PORT2, vex::gearSetting::ratio6_1,false);
vex::motor BottomIntake (vex::PORT3, vex::gearSetting::ratio18_1,false);
vex::motor FlyWheelMotor (vex::PORT7, vex::gearSetting::ratio6_1,false);
vex::motor LeftMotor (vex::PORT11, vex::gearSetting::ratio18_1,false);
vex::motor RightMotor (vex::PORT12, vex::gearSetting::ratio18_1,false);
vex::motor LeftMotor2 (vex::PORT13, vex::gearSetting::ratio18_1,false);
vex::motor RightMotor2 (vex::PORT14, vex::gearSetting::ratio18_1,false);
vex::motor Hammer (vex::PORT15, vex::gearSetting::ratio36_1,false);
vex::controller Controller1;