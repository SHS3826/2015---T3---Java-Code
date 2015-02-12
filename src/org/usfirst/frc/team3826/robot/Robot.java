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
	Joystick driveStick;
	Joystick controlStick;
	Solenoid flippers = new Solenoid(0);
	Solenoid arms = new Solenoid(1);
	Gyro lateralGyro = new Gyro(0);
	int autoLoopIncrementer, heading;
	DigitalInput entrySensor = new DigitalInput(4);
	DigitalInput exitSensor = new DigitalInput(5);
	Jaguar wenchMotor = new Jaguar(4);
	Jaguar rollerMotor = new Jaguar(5);
	Encoder wenchEncoder = new Encoder(2, 3, true);
	int saitekMultiplier, saitekXValue, saitekYValue, saitekThrottleValue, gyroTotalChange, poopcounter, time, counter, autoCounter;
	boolean saitekTriggerPulled;
	Compressor Dwayne = new Compressor(0);
	double currentHeading, dist, motorLevel, carriageHeight;
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
    	SmartDashboard.putNumber("AutoMode", 0);
    }
    
    /**
     * This function is run once each time the robot enters autonomous mode
     */
    public void autonomousInit() {
    	///autoLoopCounter = 0;
    	counter = 0;
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
    	wenchEncoder.reset();
    	if (counter != 1) {
    		counter = 1;
    		switch (SmartDashboard.getInt("AutoMode")) {
    		case 1: break;
    		case 2:	moveForward(1, .3); break;
    		case 3: moveForward(2, .15); break;
    	}
    	}
    }
    
    /**
     * This function is called once each time the robot enters teleoperated mode
     */
    public void teleopInit(){
    	lateralGyro.reset();
    	//encoderA.reset();
    	heading = 0;
    	//motorLevel = 0;
    	Dwayne.start();
    	counter = 0;
    	wenchEncoder.reset();
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        //robotDrive.setSafetyEnabled(true);  KEEP THIS OFF!!!
        while (isOperatorControl() && isEnabled()) {
        	
        	// Use the joystick X axis for lateral movement, Y axis for forward movement, and Z axis for rotation.
        	
        	/* Basic Drive Update Code:
        	robotDrive.mecanumDrive_Cartesian(controlStick.getX(), controlStick.getY(), controlStick.getThrottle(), 0); 
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
        	       	
        	if (driveStick.getRawButton(1)) {
        		if (Math.abs(driveStick.getThrottle())>=.15) {
        			robotDrive.mecanumDrive_Cartesian(driveStick.getX(), driveStick.getY(), driveStick.getThrottle(), 0);
        			lateralGyro.reset();
        		} else {
        			robotDrive.mecanumDrive_Cartesian(driveStick.getX(), driveStick.getY(), (heading)*.03, 0);
        		}
        	} else {
        		if (Math.abs(driveStick.getThrottle())>=.15) {
        			robotDrive.mecanumDrive_Cartesian(driveStick.getX()*.5, driveStick.getY()*.2, driveStick.getThrottle()*.3, 0);
        			lateralGyro.reset();
        		} else {
        			robotDrive.mecanumDrive_Cartesian(driveStick.getX()*.5, driveStick.getY()*.2, (heading)*.03, 0);
        		}
        	}

        	if (controlStick.getRawButton(9)) {rollerMotor.set(.3);} else{rollerMotor.set(0);}

        	if(Math.abs(controlStick.getY())<.1){wenchMotor.set(controlStick.getY());}

        	if (controlStick.getRawButton(10)&&counter>=10){flippers.set(!flippers.get());counter=0;}
        	counter++;
        	
        	if (controlStick.getRawAxis(3)>.5&&poopcounter>=10){arms.set(!arms.get());poopcounter=0;}
        	poopcounter++;

        	for (int i = 1; i < controlStick.getButtonCount(); i++) {
        		if (controlStick.getRawButton(i)){carriagePass(i);}
        	}
        	
        	}
        	
        	/*

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
        	
        	*/
        	
        	// This code implements FO-Drive.
        	
            //robotDrive.mecanumDrive_Cartesian(controlStick.getX()*.5, controlStick.getY()*.2, controlStick.getThrottle()*.3, lateralGyro.getAngle());
        	
			//System.out.println(encoderA.get()/497.0 + " " + motorLevel);
        	
        	//if (Math.abs(lateralGyro.getAngle()) > .5) {
        	//	heading+=lateralGyro.getAngle();
        	//}
        	heading = (int) lateralGyro.getAngle();
        	SmartDashboard.putNumber("Raw", lateralGyro.getAngle());
        	//lateralGyro.reset();
			//SmartDashboard.putNumber("Encoder Value", encoderA.get()/497.0);
			//SmartDashboard.putBoolean("Range Finder", rangeFinder.get());
			SmartDashboard.putNumber("Gyro", heading);
			/*if (encoderA.get()/497<0) {
				SmartDashboard.putBoolean("Motor", true);
			} else {
				SmartDashboard.putBoolean("Motor", false);
			}

			leftActuator.set(controlStick.getRawButton(3));
			rightActuator.set(controlStick.getRawButton(3));
          	
          	*/
          	
            Timer.delay(.04);	// wait 40ms to avoid hogging CPU cycles
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
    
    //Rotate the bot right by a passed number of degrees (Pass a negative value to turn left)
    
    public void rotateBot(int degrees) {
    	for (degrees = degrees; lateralGyro.getAngle() != degrees;) {
    		robotDrive.mecanumDrive_Cartesian(0, 0, lateralGyro.getAngle()-degrees, 0);
    	}
		robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
    }
    
    //Automatically operates the rollers to take in a new tote based on the sensors positioned on the bottom of the robot.
    
    public void acceptTote() {
    	if (!entrySensor.get()&&exitSensor.get() || autoCounter > 50) {
    		
    	} else {
    		acceptTote();
    		autoCounter++;
    	}
    }
    
    //Automatically approach tote for consuming.
    
    public void approachTote() {
    	if (entrySensor.get() || autoCounter > 25) {
    		robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
    		autoCounter = 0;
    		acceptTote();
    	} else {
    		robotDrive.mecanumDrive_Cartesian(0, .3, 0, 0);
    		autoCounter++;
    		Timer.delay(.04);
    		approachTote();
    	}
    }
    
    //An intermediary method for parsing info b/c I'm bad at Java.
    
    public void carriagePass(int x) {
    	switch (x) {
    	case 1: moveCarriage(4); break;
    	case 2: moveCarriage(17); break;
    	case 3: moveCarriage(30); break;
    	case 4: moveCarriage(43); break;
    	case 7: moveCarriage(52); break;
    	case 8: moveCarriage(0); break;
    	}
    }
    
    //Lowers or raises the carriage to a passed height.
    
    public void moveCarriage(double height)  {
    	carriageHeight = wenchEncoder.get()*3.875*3.1415926535/250;
    	if (Math.abs(carriageHeight-height) < .1) {
    		wenchMotor.set(0);
    	} else {
        	wenchMotor.set((carriageHeight-height)*.03);
    	}
    }
    
    //Actuate arms.
    
    public void actuateArms(int x) {
    	if (x == 0) {
    		arms.set(false);
    	} else if (x == 1) {
    		arms.set(true);
    	} else {
    		arms.set(!arms.get());
    	}
    }
}
