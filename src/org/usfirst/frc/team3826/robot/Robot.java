package org.usfirst.frc.team3826.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
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
	DigitalInput limitSwitch = new DigitalInput(0);
	int autoLoopIncrementer;
	int saitekMultiplier, saitekXValue, saitekYValue, saitekThrottleValue;
	boolean saitekTriggerPulled;
    final int frontLeftChannel	= 1;
    final int rearLeftChannel	= 0;
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
        	
        	/* Limit Switch Code (If the Limit Switch is activated, the motors will stop):
        	if (limitSwitch.get()) {
        			robotDrive.mecanumDrive_Cartesian(controlStick.getX(), controlStick.getY(), controlStick.getThrottle(), 0); 
        	} else {
    			robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);         		
        	}
        	*/
        	
        	// This code uses the trigger on the Saitek Controller to speed up drive speed from 25%.
        	
        	if(controlStick.getRawButton(1)) {
        		robotDrive.mecanumDrive_Cartesian(controlStick.getX(), controlStick.getY(), controlStick.getThrottle(), 0); 
        	} else {
        		robotDrive.mecanumDrive_Cartesian(controlStick.getX()*.25, controlStick.getY()*.25, controlStick.getThrottle()*.25, 0);      		
        	}
            Timer.delay(0.005);	// wait 5ms to avoid hogging CPU cycles
        }
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    	LiveWindow.run();
    }
    
}
