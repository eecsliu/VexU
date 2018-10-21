Forwards for autonomous + manual
Turning In place for autonomous + manual
Turning while moving for autonomous + manual

Function for forward autonomous

```pseudocode
def forwardAutonomous(distance):
	motor1.rotateFor(360 * distance / wheel_circumference)
	motor2.rotateFor(360 * distance / wheel_circumference)
```

Function for turning in place autonomous

```pseudocode
def turninplaceAutonomous(rotation):
	motor1.rotateFor(rotation * robot_width / wheel_diameter)
	motor2.rotateFor(-rotation * robot_width / wheel_diameter)
```

Function for manual controls


```pseudocode
def control():
    linearPower = controller.right_trigger()
    angularPower = controller.axis_1_value() * SENSITIVITY_CONSTANT
    rightPwm = leftPwm = linearPower
    leftPwm -= angularPower
    rightPwm += angularPower
    if leftPwm > 1.0
        rightPwm -= leftPwm - 1.0
        leftPwm = 1.0
    else if rightPwm > 1.0
        leftPwm -= rightPwm - 1.0
        rightPwm = 1.0
    else if leftPwm < -1.0
        rightPwm += -1.0 - leftPwm
        leftPwm = -1.0
    else if rightPwm < -1.0
        leftPwm += -1.0 - rightPwm
        rightPwm = -1.0
    if leftPwm < 0
        motor1.spin(backwards, -leftPwm, ratio)
    else
        motor2.spin(forward, leftPwm, ratio)
    if rightPwm < 0
        motor2.spin(backwards, -rightPwm, ratio)
    else
        motor2.spin(forward, rightPwm, ratio)
```

Function for autonomous turning while moving

```pseudocode
def turnMovement(xCoord, yCoord, rotation):
	turninplaceAutonomous(rotation - finalRotation)
```

API for motor actions
​	Motor.spin(directionType::fwd,50,velocityUnits::rpm);
​    Motor.rotateTo(90,rotationUnits::deg,50,velocityUnits::pct);
​    Motor.rotateFor(90,rotationUnits::deg,50,velocityUnits::pct);
​    Motor.rotateFor(2.5,timeUnits::sec,50,velocityUnits::pct);
​    Motor.startRotateTo(90,rotationUnits::deg,50,velocityUnits::pct);
​    Motor.startRotateFor(90,rotationUnits::deg,50,velocityUnits::pct);
​    Motor.stop(brakeType::coast);
API for motor sensing
​    Motor.isSpinning();
​    Motor.rotation(rotationUnits::deg);
​    Motor.velocity(velocityUnits::pct);
​    Motor.current(currentUnits::amp);
​    Motor.power(powerUnits::watts);
​    Motor.torque(torqueUnits::Nm);
​    Motor.efficiency(percentUnits::pct);
​    Motor.temperature(percentUnits::pct);