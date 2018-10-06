
package org.team3309.lib.controllers.drive.equations;

//Importing libraries made by team 3309. So, these are libraries
//by the people who wrote this code
import org.team3309.lib.KragerMath;
import org.team3309.lib.controllers.Controller;
import org.team3309.lib.controllers.statesandsignals.InputState;
import org.team3309.lib.controllers.statesandsignals.OutputSignal;
import org.usfirst.frc.team3309.driverstation.Controls;
import org.usfirst.frc.team3309.subsystems.Drive;
//Imports for hardware
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveCheezyDriveEquation extends Controller {
	//Start the drive algorithm in percent mode. This is from org.usfirst.frc.team3309.subsystems.Drive
	public DriveCheezyDriveEquation() {
		Drive.getInstance().changeToPercentMode();
	}
	//Nothing happens when the rest callback function is called
	@Override
	public void reset() {
	}
	
	private double oldWheel, quickStopAccumulator;//oldWheel stores previous values for wheel. 
	private double throttleDeadband = 0.02;//An arbitrary value to make deadzone for throttle 
	private double wheelDeadband = 0.07;//An arbitrary value to make deadzone for wheel, whatever that is
	//This method tells if the controller is done with a specific task. I think it keeps running until the task is done
	@Override
	public OutputSignal getOutputSignal(InputState inputState) {
		double throttle = Controls.driverController.getY(Hand.kLeft);//Get the y-component of XboxController(0) left joystick
		double wheel = Controls.driverController.getX(Hand.kRight);//Get the x-component of XboxController(0) right joystick
		boolean isQuickTurn = Controls.driverController.getBumper(Hand.kRight);//Gets whether or not the right bumer is pressed
		boolean isHighGear = true;
		OutputSignal signal = new OutputSignal();//Output signal manages the power of the right and left motors
		double wheelNonLinearity;

		wheel = handleDeadband(wheel, wheelDeadband);//Calculate the initial value for wheel
		throttle = handleDeadband(throttle, throttleDeadband);//Calculate the initial value for throttle

		double negInertia = wheel - oldWheel;//negInertia is the change in wheel values
		oldWheel = wheel;//Update what oldWheel is

		//Using a constant called wheelNonLinearity, run wheel through a math equation 2 or 3 times
		if (isHighGear) {
			wheelNonLinearity = 0.6;
			// Apply a sin function that's scaled to make it feel better.
			wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / Math.sin(Math.PI / 2.0 * wheelNonLinearity);
			wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / Math.sin(Math.PI / 2.0 * wheelNonLinearity);
		} else {
			wheelNonLinearity = 0.5;
			// Apply a sin function that's scaled to make it feel better.
			wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / Math.sin(Math.PI / 2.0 * wheelNonLinearity);
			wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / Math.sin(Math.PI / 2.0 * wheelNonLinearity);
			wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / Math.sin(Math.PI / 2.0 * wheelNonLinearity);
		}

		double leftPwm, rightPwm, overPower;
		double sensitivity = .3;

		double angularPower;//angularPower is positive to turn right, negative to turn left
		double linearPower;

		// Negative inertia!
		double negInertiaAccumulator = 0.0;
		double negInertiaScalar = 5;
		//I have no idea why there is an if statement to possibly make neginertiaScalar the same thing
		if (isHighGear) {
			negInertiaScalar = 5.0;
		} else {
			/*
			 * if (wheel * negInertia > 0) { negInertiaScalar = 1.75; } else {
			 * if (Math.abs(wheel) > 0.65) { negInertiaScalar = 4.0; } else {
			 * negInertiaScalar = 3.0; } } sensitivity = .6;
			 */
		}
		double negInertiaPower = negInertia * negInertiaScalar;//Multiply negativeInertia declared on line 42 by the constant
		negInertiaAccumulator += negInertiaPower;//Add previous neg inertia 

		//Adds previous negativeInertiaPowers to the wheel
		wheel = wheel + negInertiaAccumulator;
		//Changes negInertiaAccumulator towards 0
		if (negInertiaAccumulator > 1) {
			negInertiaAccumulator -= 1;
		} else if (negInertiaAccumulator < -1) {
			negInertiaAccumulator += 1;
		} else {
			negInertiaAccumulator = 0;
		}
		linearPower = throttle;
		//When the throttle is high, reduce the sensitivity of turns
		if (throttle > .4) {
			sensitivity = .15;
		} else {
			sensitivity = .3;
		}
		// Quickturn!
		if (isQuickTurn) {
			//When the throttle is very low, quickStopAccumulator = 95 percent the previous value + 25 percent the value 
			//of wheel limited to 1
			//idk where the first assignment of quickStopAccumulator happens
			if (Math.abs(linearPower) < 0.2) {
				double alpha = 0.05;
				quickStopAccumulator = (1 - alpha) * quickStopAccumulator + alpha * limit(wheel, 1.0) * 5;
			}
			overPower = 1.0;
			//This if statement is pretty useless
			if (isHighGear) {
				sensitivity = .455;
			} else {
				sensitivity = .455;
			}
			//Sets the turning equal to the value of wheel which is based off the right joystick 
			angularPower = wheel * sensitivity;
			if (wheel == 1 && wheel == -1) {
				angularPower = wheel;//If joystick is fully in a direction, just set angular power equal to it
			}
		} else {
			overPower = 0.0;
			//Sets the power of the turn equal to how much fast the vehicle is going * right joystick * sensitivity variable - a value that never exists
			angularPower = Math.abs(throttle) * wheel * sensitivity - quickStopAccumulator;
			if (quickStopAccumulator > 1) {
				quickStopAccumulator -= 1;
			} else if (quickStopAccumulator < -1) {
				quickStopAccumulator += 1;
			} else {
				quickStopAccumulator = 0.0;
			}
		}
		rightPwm = leftPwm = linearPower;//Set both powers to be the same based off throttle
		leftPwm -= angularPower;//Adjust left and right power based off how much turning
		rightPwm += angularPower;
		//If the power of either left or right wheel are too high, set it lower as well as scaling down the other wheel so the turning is the same
		if (leftPwm > 1.0) {
			rightPwm -= overPower * (leftPwm - 1.0);
			leftPwm = 1.0;
		} else if (rightPwm > 1.0) {
			leftPwm -= overPower * (rightPwm - 1.0);
			rightPwm = 1.0;
		} else if (leftPwm < -1.0) {
			rightPwm += overPower * (-1.0 - leftPwm);
			leftPwm = -1.0;
		} else if (rightPwm < -1.0) {
			leftPwm += overPower * (-1.0 - rightPwm);
			rightPwm = -1.0;
		}
		signal.setLeftMotor(leftPwm);//Sets what the left motor power is
		signal.setRightMotor(rightPwm);//Sets what the right motor power is
		SmartDashboard.putNumber("left", leftPwm);//Update the screen
		SmartDashboard.putNumber("right", rightPwm);//Update the screen
		return signal;
	}

	@Override
	public boolean isCompleted() {
		return false;
	}
	//Returns val if it is greater than deadband. Otherwise return 0
	public double handleDeadband(double val, double deadband) {
		return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
	}
	//Return v if it is less than limit. Otherwise, return -1 if v is negative or +1 if v is positive
	public static double limit(double v, double limit) {
		return (Math.abs(v) < limit) ? v : limit * (v < 0 ? -1 : 1);
	}
}
