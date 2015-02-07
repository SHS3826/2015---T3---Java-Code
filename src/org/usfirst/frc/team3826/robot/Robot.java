package org.usfirst.frc.team3826.robot;

import java.lang.Math;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	RobotDrive robotDrive;
	Joystick controlStick;
	DigitalInput rangeFinder = new DigitalInput(2);
	Solenoid leftActuator = new Solenoid(0);
	Solenoid rightActuator = new Solenoid(1);
	Gyro lateralGyro = new Gyro(0);
	AnalogInput distDet = new AnalogInput(1);
	int autoLoopIncrementer, heading;
	Victor encodedMotor = new Victor(4);
	Encoder encoderA = new Encoder(0, 1, true);
	int saitekMultiplier, saitekXValue, saitekYValue, saitekThrottleValue, gyroTotalChange, time;
	boolean saitekTriggerPulled;
	double currentHeading, dist, motorLevel;
    final int frontLeftChannel	= 0;
    final int rearLeftChannel	= 1;
    final int frontRightChannel	= 2;
    final int rearRightChannel	= 3;
    final int joystickChannel	= 0;
    boolean [] limitswitches = {};
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    
    public void robotInit() {
    	controlStick = new Joystick(0);
        robotDrive = new RobotDrive(frontLeftChannel, rearLeftChannel, frontRightChannel, rearRightChannel);
        robotDrive.setSafetyEnabled(true);
    	robotDrive.setExpiration(10);
    	robotDrive.setInvertedMotor(MotorType.kFrontRight, true);	// invert the left side motors
    	robotDrive.setInvertedMotor(MotorType.kRearRight, true);	// you may need to change or remove this to match your robot
    	//SmartDashboard.putString("AutoMode", "None");
    }
    
    /**
     * This function is run once each time the robot enters autonomous mode
     */
    public void autonomousInit() {
    	///autoLoopCounter = 0;
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	//if(autoLoopCounter < 100) //Check if we've completed 100 loops (approximately 2 seconds)
		//{
		//	myRobot.drive(-0.5, 0.0); 	// drive forwards half speed
		//	autoLoopCounter++;
		//	} else {
		//	myRobot.drive(0.0, 0.0); 	// stop robot
		//}
    }
    
    /**
     * This function is called once each time the robot enters tele-operated mode
     */
    public void teleopInit(){
    	lateralGyro.reset();
    	currentHeading = 0;
    	encoderA.reset();
    	heading = 0;
    	motorLevel = 0;
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        //robotDrive.setSafetyEnabled(true);
        while (isOperatorControl() && isEnabled()) {
        	
        	
        	// Use the joystick X axis for lateral movement, Y axis for forward movement, and Z axis for rotation.
        	// This sample does not use field-oriented drive, so the gyro input is set to zero.
        	// If the trigger on the Saitek controller is pulled, the movement will be much slower.
        	
        	/* Basic Drive Update Code:
        	robotDrive.mecanumDrive_Cartesian(controlStick.getX(), controlStick.getY(), controlStick.getThrottle(), 0); 
        	*/
        	
 			// Limit Switch Code (If the Limit Switch is activated, the motors will stop):
        	
        	/*
        	
			if (limitSwitch.get()) {        		
        		robotDrive.mecanumDrive_Cartesian(Math.pow(controlStick.getX(),3), Math.pow(controlStick.getY(),3), Math.pow(controlStick.getThrottle(),3), lateralGyro.getAngle()); 
        	} else {
    			robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);         		
        	}
        	
        	*/
        	
        	// This code is the Straight-Drive Code, but with axis changes for the Xbox Controller.
        	
        	/*
        	
        	if (controlStick.getRawButton(1)) {
        		if (Math.abs(controlStick.getX())>=.15) {
        			robotDrive.mecanumDrive_Cartesian(controlStick.getRawAxis(4), controlStick.getY(), controlStick.getX(), 0);
        			lateralGyro.reset();
        		} else {
        			robotDrive.mecanumDrive_Cartesian(controlStick.getRawAxis(4), controlStick.getY(), -(lateralGyro.getAngle())*.03, 0);
        		}
        	} else {
        		if (Math.abs(controlStick.getX())>=.15) {
        			robotDrive.mecanumDrive_Cartesian(controlStick.getRawAxis(4)*.5, controlStick.getY()*.2, controlStick.getX()*.3, 0);
        			lateralGyro.reset();
        		} else {
        			robotDrive.mecanumDrive_Cartesian(controlStick.getRawAxis(4)*.5, controlStick.getY()*.2, -(lateralGyro.getAngle())*.03, 0);
        		}
        	}
        	
        	*/
        	
        	//This code drives the robot in a (mostly) straight line, as well as uses Multi-Speed Drive.
        	
        	       	
        	if (controlStick.getRawButton(1)) {
        		if (Math.abs(controlStick.getThrottle())>=.15) {
        			robotDrive.mecanumDrive_Cartesian(controlStick.getX(), controlStick.getY(), controlStick.getThrottle(), 0);
        			lateralGyro.reset();
        		} else {
        			robotDrive.mecanumDrive_Cartesian(controlStick.getX(), controlStick.getY(), -(heading)*.03, 0);
        		}
        	} else {
        		if (Math.abs(controlStick.getThrottle())>=.15) {
        			robotDrive.mecanumDrive_Cartesian(controlStick.getX()*.5, controlStick.getY()*.2, controlStick.getThrottle()*.3, 0);
        			lateralGyro.reset();
        		} else {
        			robotDrive.mecanumDrive_Cartesian(controlStick.getX()*.5, controlStick.getY()*.2, -(heading)*.03, 0);
        		}
        	}
        	
        	
        	
        	encodedMotor.set(controlStick.getZ());
        	if (controlStick.getRawButton(1)) {
        		motorLevel = 1;
        	} else if (controlStick.getRawButton(2)) {
        		motorLevel = 0;
        	} else if (controlStick.getRawButton(3)) {
        		motorLevel = -1;
        	} else if (controlStick.getRawButton(4)) {
        		motorLevel = 5;
        	}
        	
        	/*
        	if (Math.abs(controlStick.getZ() < .1)) {
        	if (encoderA.get()/497 < (motorLevel * 1.0)) {
        		encodedMotor.set(encoderA.get()/497 - (motorLevel * 1.0));
        	} else if (encoderA.get()/497 > (motorLevel * 1.0)) {
        		encodedMotor.set(encoderA.get()/497 + (motorLevel * 1.0));
        	} else {
        		encodedMotor.set(0);
        	}
        	
        	*/
        	
        	encodedMotor.set(4*(encoderA.get()/497.0 - (motorLevel * 1.0)));
        	
        	//}
        	
        	// This code implements FO-Drive.
        	
            //robotDrive.mecanumDrive_Cartesian(controlStick.getX()*.5, controlStick.getY()*.2, controlStick.getThrottle()*.3, lateralGyro.getAngle());
        	
			//System.out.println(encoderA.get()/497.0 + " " + motorLevel);
        	
        	//if (Math.abs(lateralGyro.getAngle()) > .5) {
        	//	heading+=lateralGyro.getAngle();
        	//}
        	heading = (int) lateralGyro.getAngle();
        	SmartDashboard.putDouble("Raw", lateralGyro.getAngle());
        	//lateralGyro.reset();
			SmartDashboard.putNumber("Encoder Value", encoderA.get()/497.0);
			SmartDashboard.putBoolean("Range Finder", rangeFinder.get());
			SmartDashboard.putInt("Gyro", heading);
			if (encoderA.get()/497<0) {
				SmartDashboard.putBoolean("Motor", true);
			} else {
				SmartDashboard.putBoolean("Motor", false);
			}

			leftActuator.set(controlStick.getRawButton(3));
			rightActuator.set(controlStick.getRawButton(3));
          	
            Timer.delay(.04);	// wait 40ms to avoid hogging CPU cycles
        }
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    	LiveWindow.run();
    }

    //Move forward based on time-based dead reckoning. Pass a negative speed value for moving backwards.
    
    public void moveForward(int seconds, double speed) {
		robotDrive.mecanumDrive_Cartesian(0, speed, 0, 0);
		Timer.delay(seconds);
		robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
    }

    //Move right based on time-based dead reckoning. Pass a negative speed value for moving left.
    
    public void moveSideways(int seconds, double speed) {
		robotDrive.mecanumDrive_Cartesian(speed, 0, 0, 0);
		Timer.delay(seconds);
		robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
    }
    
    public void rotateBot(int degrees) {
    	for (degrees = degrees; lateralGyro.getAngle() != degrees;) {
    		robotDrive.mecanumDrive_Cartesian(0, 0, lateralGyro.getAngle()-degrees, 0);
    	}
		robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
    }
}
