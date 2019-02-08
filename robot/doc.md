# Pseudocode

Function for forward autonomous

```pseudocode
def forwardAutonomous(distance):
	motor1.rotateFor(360 * distance / wheel_circumference)
	motor2.rotateFor(360 * distance / wheel_circumference)
```

Function for turning in place autonomous

```pseudocode
def turninplaceAutonomous(rotation):
	update_orientation()//Pseudocode from below
	target_orientation = orientation + rotation
	
	motor1.rotateFor(rotation * robot_width / wheel_diameter)
	motor2.rotateFor(-rotation * robot_width / wheel_diameter)
	
	update_orientation()
	error_sum, last_error, current_error = 0, orientation - target_orientation, orientation - target_orientation
	pid_sum = P_CONSTANT * last_error + I_CONSTANT * error_sum//Don't need derivative 
	while pid_sum > MINIMUM_ERROR_CONSTANT:
		activate_motors(pid_sum)
        update_orientation()
        error_sum, last_error, current_error = error_sum + current_error, current_error, orientation - target_orientation
        pid_sum = P_CONSTANT * last_error + I_CONSTANT * error_sum + D_CONSTANT * 
	
	
```

Function for manual controls


```pseudocode
def control():
    linearPower = controller.right_trigger() - controller.left_trigger()
    angularPower = controller.axis_1_value() * SENSITIVITY_CONSTANT
    activate_motors(angularPower)
def activate_motors(angularPower, linearPower = 0)
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

Function for lifting
```pseudocode
lift_motors = [motors]
def liftArm(height):
	if height == LOW_HEIGHT:
		liftHelperFunction(low_switch, direction::fwd)
	else:
		liftHelperFunction(high_switch, direction::fwd)
def resetArm():
	liftHelperFunction(reset_switch, direction::backward)
def liftHelperFunction(switch, direction):
	while(not switch.is_pressed()):
		for motor in lift_motors:
			motor.spin(direction, 50)
```

Functions for intake toggle

```pseudocode
def toggle_slow():
	if not motor.isSpinning() or not motor.velocity == SLOW_SPEED
		motor.spin(directionType::fwd, SLOW_SPEED)
	else:
		motor.stop()

def toggle_fast():
	if not motor.isSpinning() or not motor.velocity == FAST_SPEED
		motor.spin(directionType::fwd, FAST_SPEED)
	else:
		motor.stop()
		
def toggle_slow_back():
	if not motor.isSpinning() or not motor.velocity == SLOW_SPEED
		motor.spin(directionType::back, SLOW_SPEED)
	else:
		motor.stop()

def toggle_fast_back():
	if not motor.isSpinning() or not motor.velocity == FAST_SPEED
		motor.spin(directionType::back, FAST_SPEED)
	else:
		motor.stop()
```



Control Function for orientation

```pseudocode
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
	accelOffset = lsm.accelData - accelZero
	magOffset = lsm.magData - magZero
	gyroOffset = lsm.gyroData - gyroZero
	orientation = (magOffset.x +gyroOffset.y) / 2
	
```
Large Robot Autonomous strategy:
```pseudocode
forwardAutonomous(distance) toggle_fast(forward) acquire_cap() toggle_fast(forward) turninplaceAutonomous(-90) forwardAutonomous(distance) turninplaceAutonomous(-270) liftArm(HIGH) forwardAutonomous(distance) resetArm() forwardAutonomous(-distance) turninplaceAutonomous(90) forwardAutonomous(distance) turninplaceAutonomous(90) forwardAutonomous(distance) acquire_cap() turninplaceAutonomous(180) forwardAutonomous(distance) turninplaceAutonomous(90) liftArm(HIGH) forwardAutonomous(distance) resetArm() turninplaceAutonomous(-90) forwardAutonomous(distance) turninplaceAutonomous(-90) forwardAutonomous(distance) aim() shoot() aim() shoot() turninplaceAutonomous(-100) forwardAutonomous(distance)
```
Small Robot Autonomous strategy:
```pseudocode
forwardAutonomous(distance) toggle_fast(forward) turninplaceAutonomous(90) forwardAutonomous(distance) turninplaceAutonomous(90) forwardAutonomous(distance) acquire_cap() turninplaceAutonomous(35) forwardAutonomous(distance) lift_arm(HIGH) forwardAutonomous(distance) resetArm() turninplaceAutonomous(180) forwardAutonomous(distance) turninplaceAutonomous(180) forwardAutonomous(distance) turninplaceAutonomous(-90) forwardAutonomous(distance) turninplaceAutonomous(180) forwardAutonomous(distance) turninplaceAutonomous(90) delay(time) aim() shoot() turninPlaceAutonomous(90) aim() shoot()
```

Functions for Aiming:
```pseudocode
getDistance()
getOrientationChange()
calculatePower(distance)
def aim():	
	turninplaceAutonomous(getOrientationChange())
	
def shoot():
	calculatePower(getDistance())
	place_ball()
```
API for motor actions

* Motor.spin(directionType::fwd,50,velocityUnits::rpm);
* Motor.rotateTo(90,rotationUnits::deg,50,velocityUnits::pct);
* Motor.rotateFor(90,rotationUnits::deg,50,velocityUnits::pct);
* Motor.rotateFor(2.5,timeUnits::sec,50,velocityUnits::pct);
* Motor.startRotateTo(90,rotationUnits::deg,50,velocityUnits::pct);
* Motor.startRotateFor(90,rotationUnits::deg,50,velocityUnits::pct);
* Motor.stop(brakeType::coast);

API for motor sensing

* Motor.isSpinning();
* Motor.rotation(rotationUnits::deg);
* Motor.velocity(velocityUnits::pct);
* Motor.current(currentUnits::amp);
* Motor.power(powerUnits::watts);
* Motor.torque(torqueUnits::Nm);
* Motor.efficiency(percentUnits::pct);
* Motor.temperature(percentUnits::pct);

# Notes from design review:

Cascade lift works by spinning motors that pull chains that lift up

* Left to right around 8 inches

* Depth 4 inches

* Each stage lifts 12 inch

* 4 motors

* **12 links per rotation, 1/2 inch per link = 6 inches per rotation**

Reversed Bar uses parallelograms to lift

* Really tall
* 4 motors
* Takes up more space

6 wheel tank has 2 grippy in the middle and 4 omniwheels

* All the wheels are driven together with sprockets and chains
* 4 inch wheels? They might be 4.25 inches cause vex is weird
* Motors are on rear wheel to enable more stuff at the front but motor on center wheel would be better cause if one of the chains breaks, you still have 2 wheels on the side
* 2 motors on each side

Double wheel launcher

* 2 wheels + 2 motors
* Can launch sideways by having one motor faster than the other
* Can only launch 2 meters per second with 600 rpm motors
* Probably will have gearing
* Adjust height by changing speed of motors

Single wheel launcher

* Needs a gear box
* Metal acts as ramp to launch ball
* Paddle starting position makes it inconsistent / not really repeatable
* Can't aim sideways
* "Cantilever beam"

Vertical Intake + Tread intake

* Smaller input area
* 2 motors

Rubber band intake

* One motor
* Gearing in case you want it to go fast
* If it's slower, can be used to flip caps
* Might spin at around 2x the speed of the drivetrain

Rotating double forklift (passive)

* Kind of just runs into the cap
* 2 motors, one to fold up the system for storage and one to rotate the forklift to flip caps
* Can pick up tilted caps only if driver approaches from the side

Rotating double forklift (active)

* Pretty much the same. Just the top can fold up and down
* Can pick up caps in any angle whereas passive cannot

Teeth active forklift

* Only needs 2 motors
* Around 1 pound
* Latches onto the edge of the cap
* Kind of high torque
* Can manipulate caps from any angle except 'when cap is angled towards you'
* Can't skim the ground

Cap flipper

* Only needs one motor
* Can only do high scoring caps at the posts
* Lightest

# Controls

Buttons from left to right, top to bottom:

* Button 1: Intake fast and off toggle
* Button 2:
* Button 3:
* Button 4: Intake slow and off toggle to flip caps
* Button 5: Toggle ball launcher
* Button 6: Move cap flipper down
* Button 7: Move cap flipper up
* Button 8:

L1: Aim using COMPUTER SCIENCE

L2:

R1: Shoot using ball

R2: Throttle

X-axis Left: Turning

Y-axis Left:

X-axis Right: 

Y-axis Right: Moving the lift up and down