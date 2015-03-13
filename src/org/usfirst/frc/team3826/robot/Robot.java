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
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.PIDController;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	RobotDrive robotDrive;
	Joystick driveStick = new Joystick(1);
	Joystick controlStick = new Joystick(0);
	//Solenoid flippers = new Solenoid(0);
	//Solenoid arms = new Solenoid(1);
	Gyro lateralGyro = new Gyro(1);
	int autoLoopIncrementer, heading;
	DigitalInput limitSwitchBot = new DigitalInput(0);
	DigitalInput limitSwitchTop = new DigitalInput(1);
	DigitalInput photoSwitchFront = new DigitalInput(4);
	DigitalInput photoSwitchEntry = new DigitalInput(5);
	DigitalInput photoSwitchExit = new DigitalInput(6);
	Jaguar wenchMotor = new Jaguar(4);
	Jaguar rollerMotor = new Jaguar(5); 
	Servo leftServo = new Servo(6);
	Servo rightServo = new Servo(7);
	Servo leftBServo = new Servo(8);
	Servo rightBServo = new Servo(9);
	PowerDistributionPanel PDP = new PowerDistributionPanel();
	Encoder wenchEncoder = new Encoder(7, 8, true);
	//CameraServer frontCamera;
	int saitekMultiplier, saitekXValue, saitekYValue, saitekThrottleValue, gyroTotalChange, poopcounter, time, counter, autoCounter, botWench, totWench, binWench, topWench;
	int [] encoderCounter;
	boolean saitekTriggerPulled, eject, intake;
	Compressor Dwayne = new Compressor(0);
	String wenchStatus = "stay";
	double wenchRate, currentHeading, dist, motorLevel, carriageHeight, rollerSpeed, wantedHeight, Proportion, Integral, Derivative;
    final int frontLeftChannel	= 0;
    final int rearLeftChannel	= 1;
    final int frontRightChannel	= 2;
    final int rearRightChannel	= 3;
	PIDController wenchControl = new PIDController(0.002, 0.0, -0.001, wenchEncoder, wenchMotor);
    //final int joystickChannel	= 0;
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    
    public void robotInit() {
        robotDrive = new RobotDrive(frontLeftChannel, rearLeftChannel, rearRightChannel, frontRightChannel);
        robotDrive.setSafetyEnabled(true);
    	robotDrive.setExpiration(10);
    	wenchControl.enable();
    	robotDrive.setInvertedMotor(MotorType.kFrontRight, true);	// invert the left side motors
    	robotDrive.setInvertedMotor(MotorType.kRearRight, true);	// you may need to change or remove this to match your robot
    	SmartDashboard.putNumber("AutoMode", 0);
    	botWench = 0;
    	totWench = 0;
    	binWench = 0;
    	topWench = 7414;
    	//frontCamera = CameraServer.getInstance();
    	//frontCamera.setQuality(50);
    	//frontCamera.startAutomaticCapture("cam0");
    }
    
    /**
     * This function is run once each time the robot enters autonomous mode
     */
    public void autonomousInit() {
    	///autoLoopCounter = 0;
    	counter = 0;
    	autoCounter = 0;
    	wenchEncoder.reset();
    	lateralGyro.reset();
    }

    /**
     * This function is called periodically during autonomous
     */
	public void autonomousPeriodic() {
		
		//Preliminary code to reset
		
		//USE ME TO MOVE CARRIAGE:
		//wenchControl.setSetpoint( - INSERT HEIGHT HERE - );
		
    	/*wenchEncoder.reset();
    	if (counter < 190) {
    		robotDrive.mecanumDrive_Cartesian(0, -.3, 0, 0);
    		counter++;
    	} else {
    		robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
    	}*/
		if (counter == 0) {
    		robotDrive.mecanumDrive_Cartesian(0, .3, lateralGyro.getAngle()*.03, 0);
		    autoCounter++;
		}
		if ((photoSwitchFront.get() || autoCounter > 190) && counter == 0) {
			counter = 1;
			robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
		}
		if (counter == 1) {
	    		if(photoSwitchFront.get() && !photoSwitchEntry.get()) {
	    		rollerMotor.set(-.5);
	    		} else if(photoSwitchFront.get() && photoSwitchEntry.get()) {
	    			rollerMotor.set(-.3);
	    		}
		}
		if (counter == 1 && !photoSwitchFront.get()) {
    		rollerMotor.set(0);
    		counter = 2;
    		autoCounter = 0;
    	}
		if (counter == 2) {
			robotDrive.mecanumDrive_Cartesian(.5, 0, lateralGyro.getAngle()*.03, 0);
		autoCounter++;
		}
		if (autoCounter >=150) {
			robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
			counter = 3;
		}
    	}
    	
    
    /**
     * This function is called once each time the robot enters teleoperated mode
     */
    public void teleopInit(){
    	lateralGyro.reset();
    	heading = 0;
    	Dwayne.start();
    	counter = 0;
    	wenchEncoder.reset();
    	wenchEncoder.reset();
    	rollerSpeed = 0;
    	wenchControl.setOutputRange(-1, .6);
    	wenchEncoder.setDistancePerPulse(1);
    	wantedHeight = 0;
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
        			robotDrive.mecanumDrive_Cartesian(controlStick.getRawAxis(4), controlStick.getY(), (lateralGyro.getAngle())*.03, 0);
        		}
        	} else {
        		if (Math.abs(controlStick.getX())>=.15) {
        			robotDrive.mecanumDrive_Cartesian(controlStick.getRawAxis(4)*.5, controlStick.getY()*.2, controlStick.getX()*.3, 0);
        			lateralGyro.reset();
        		} else {
        			robotDrive.mecanumDrive_Cartesian(controlStick.getRawAxis(4)*.5, controlStick.getY()*.2, (lateralGyro.getAngle())*.03, 0);
        		}
        	}
        	
        	//*/
        	
        	//This code drives the robot in a (mostly) straight line, as well as uses Multi-Speed Drive.
        	
        	//if (controlStick.getRawButton(4) || (eject == true)) {
        	//	rollerMotor.set(-.3);
        	//} else {
        	//rollerMotor.set(0);}
        	
        	/*if (driveStick.getRawButton(1)) {
        			robotDrive.mecanumDrive_Cartesian(driveStick.getX(), driveStick.getY(), driveStick.getThrottle(), 0);
        			lateralGyro.reset();
        	
        	} else {*/
        	
        		if (Math.abs(driveStick.getThrottle())>=.10) {
        			robotDrive.mecanumDrive_Cartesian(Math.pow(driveStick.getX(), 3)*.6, Math.pow(driveStick.getY(), 3)*.4, Math.pow(driveStick.getThrottle(), 3)*.5, 0);
        			lateralGyro.reset();
        		} else {
        			robotDrive.mecanumDrive_Cartesian(Math.pow(driveStick.getX(), 3)*.6, Math.pow(driveStick.getY(), 3)*.4, lateralGyro.getAngle()*.03, 0);
        		}

            if (controlStick.getRawButton(5)&&controlStick.getRawButton(9)) {
            	wantedHeight = 0;
            }
            if (controlStick.getRawButton(6)&&controlStick.getRawButton(9)) {
            	wantedHeight = -2489;
            }
            if (controlStick.getRawButton(7)&&controlStick.getRawButton(9)) {
            	wantedHeight = -5000;
            }
            if (controlStick.getRawButton(8)&&controlStick.getRawButton(9)) {
            	wantedHeight = -1600;
            }
        if (controlStick.getRawButton(4)&&controlStick.getRawButton(9)) {
        	wantedHeight = 7414;
        }
    	if(controlStick.getRawButton(1)&&!controlStick.getRawButton(9)) {
    		rollerSpeed -= .06;
    	} else {
    		rollerSpeed += .8;
    	}
    	if (rollerSpeed > 0 ) {
    		rollerSpeed = 0;
    	} else if (rollerSpeed < -.7) {
    		rollerSpeed = -.7;
    	}

    	if (!controlStick.getRawButton(9)&& controlStick.getRawButton(3)) {
    		leftServo.set(0);
    		rightServo.set(0);
    	}

    	if (controlStick.getRawButton(9)&& controlStick.getRawButton(3)) {
    		leftServo.set(.6);
    		rightServo.set(.6);
    	}
    	
    	wenchRate = 0;
    	if (controlStick.getRawButton(1)&&controlStick.getRawButton(9)&&controlStick.getRawButton(2)) {
    		wenchEncoder.reset();
    	}

    	if (controlStick.getRawButton(2)) {
    		if (intake == false && photoSwitchEntry.get()) {
    			leftServo.set(.6);
    			rightServo.set(.6);
    			if (wenchEncoder.get()< 0 && counter < 20) {
    				wantedHeight = 0;
    			} else
    			if (wenchEncoder.get() > -10) {
    				counter++;
    			}
    			if (counter == 20) {
    				wantedHeight = -2489;
   				}
    		}
    	}
    	
    	if (wenchEncoder.get() < -2000) {
    		counter = 0;
    	}
    	
    	if (!photoSwitchFront.get()) {
    	    		intake = false;
    	    	} else
    	if (((photoSwitchFront.get() && !photoSwitchEntry.get()))) {
        		intake = true;
        }
    	
    	if (intake == true) {
    		if(photoSwitchFront.get() && !photoSwitchEntry.get()) {
    		rollerMotor.set(-.5);
    		} else if(photoSwitchFront.get() && photoSwitchEntry.get()) {
    			rollerMotor.set(-.3);
    		}
    	} else {
    		rollerMotor.set(rollerSpeed);
    	}
    	
    	wenchControl.setSetpoint(wantedHeight);
    	
    	/*if (!limitSwitchBot.get()) {
    		if (wantedHeight>wenchEncoder.get()) {
    			wenchControl.setSetpoint(0);
    		} else {
    			wenchControl.setSetpoint(wantedHeight);
    		}
    	}
    	
    	if (!limitSwitchTop.get()) {
    		if (wantedHeight<wenchEncoder.get()) {
    			wenchControl.setSetpoint(0);
    		} else {
    			wenchControl.setSetpoint(wantedHeight);
    		}
    	}*/

        	SmartDashboard.putBoolean("FrontSensor", photoSwitchFront.get());
        	SmartDashboard.putBoolean("EntrySensor", photoSwitchEntry.get());
        	SmartDashboard.putBoolean("TopSwitch", limitSwitchTop.get());
        	SmartDashboard.putBoolean("BottomSwitch", limitSwitchBot.get());
        	SmartDashboard.putNumber("POV", controlStick.getPOV());        	
        	heading = (int) lateralGyro.getAngle();
        	SmartDashboard.putNumber("Raw", lateralGyro.getAngle());
			SmartDashboard.putNumber("Gyro", heading);
			SmartDashboard.putNumber("Wench Voltage", PDP.getCurrent(4));
			SmartDashboard.putNumber("Encoder", wenchEncoder.get());
			carriageHeight = -wenchEncoder.get();
			SmartDashboard.putNumber("Carriage Height", carriageHeight);
          	
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
    	if(counter<=(Math.sqrt(seconds)/50)) {
		robotDrive.mecanumDrive_Cartesian(0, speed, lateralGyro.getAngle()*.03, 0);
		Timer.delay(Math.sqrt(seconds)/50);
		robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
    	}
    }

    //Move right based on time-based dead reckoning. Pass a negative speed value for moving left.
    
    public void moveSideways(int seconds, double speed) {
    	if(counter<=(Math.sqrt(seconds)/50)) {
		robotDrive.mecanumDrive_Cartesian(speed, 0, (lateralGyro.getAngle())*.03, 0);
		Timer.delay(seconds);
		robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
    	}
    }
    
    //Rotate the bot right by a passed number of degrees (Pass a negative value to turn left)
    
    public void rotateBot(int degrees) {
    	for (degrees = degrees; lateralGyro.getAngle() != degrees;) {
    		robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
    	}
		robotDrive.mecanumDrive_Cartesian(0, 0, 0, 0);
		lateralGyro.reset();
    }
    
    //Automatically operates the rollers to take in a new tote based on the sensors positioned on the bottom of the robot.
    
    public void acceptTote() {
    	if (!photoSwitchFront.get() || autoCounter > 250) {
    		rollerMotor.set(0);
    	} else {
    		acceptTote();
    		autoCounter++;
    		if(photoSwitchFront.get() && !photoSwitchEntry.get()) {
    		rollerMotor.set(-.5);
    		} else if(photoSwitchFront.get() && photoSwitchEntry.get()) {
    			rollerMotor.set(-.3);
    		}
    		Timer.delay(20);
    	}
    }
    
    //Automatically approach tote for consuming.
    
    /*
    
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
    
    */
    
    //An intermediary method for parsing info b/c I'm bad at Java.
    
    /*public void carriagePass(int x) {
    	switch (x) {
    	case 1: moveCarriage(0); break; //Bottom
    	case 2: moveCarriage(2048); break;
    	case 3: moveCarriage(0); break;
    	case 4: moveCarriage(7414); break;
    	}
    }
    
    //Lowers or raises the carriage to a passed height in encoder count.
    
    public void moveCarriage(double height)  {
    	wenchControl.setSetpoint(wantedHeight);
    }*/
    
    //Actuate arms.
    
    /*
    
    public void actuateArms(int x) {
    	if (x == 0) {
    		arms.set(false);
    	} else if (x == 1) {
    		arms.set(true);
    	} else {
    		arms.set(!arms.get());
    	}
    }
    
    */
}