
package org.usfirst.frc.team2850.robot;

import org.usfirst.frc.team2850.subsystems.Shooter;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.PIDSubsystem;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Encoder;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
    public static Joystick xbox1;
	
	public static RobotDrive drivetrain;
	public static RobotDrive drivetrain2;
	
	public static PowerDistributionPanel pdp;
	
	public static Spark leftDrive1;
	public static Spark rightDrive1;
	public static Spark leftDrive2;
	public static Spark rightDrive2;
	public static Spark leftDrive3;
	public static Spark rightDrive3;
	public static Compressor compressor;
	public static Solenoid driveshifter;
	
	public static Encoder leftEncoder;
	public static Encoder rightEncoder;
	
	public static double pDrive;
	public static double iDrive;
	public static double dDrive;
	
	public static int timeX;
	
	PIDController driveController;
	
    public static boolean high;
    //how many pulses per rotation? how many feet per rotation? (gearing, tires) needs to be decided
//    public static double DIST_PER_PULSE = .2;
//   	
//    public static double Kp = .005; //proportional gain
//    public static double Ki = .005; //integral gain
//    public static double Kd = .005; //derivative gain
//    
//    public static double PID_SETPOINT = 10.0; //10 feet
//    public static double TARGET_THRESHOLD = .25; //within .25 ft is ok
//    
//   	public static PID linearDriveLeft = new PID(Kp, Ki, Kd, PID_SETPOINT, leftEncoder.getDistance());
//   	public static PID linearDriveRight = new PID(Kp, Ki, Kd, PID_SETPOINT, rightEncoder.getDistance());
   	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	xbox1=new Joystick(0);
    	leftDrive1=new Spark(0);
    	leftDrive2=new Spark(1);
    	leftDrive3=new Spark(2);
    	rightDrive1=new Spark(3);
    	rightDrive2=new Spark(4);
    	rightDrive3=new Spark(5);
    	
    	drivetrain= new RobotDrive(leftDrive1, leftDrive2, rightDrive1, rightDrive2);
    	drivetrain2= new RobotDrive(leftDrive3, rightDrive3);
    	
    	compressor = new Compressor();
    	driveshifter=new Solenoid(0);
    	
    	timeX = 0;
    	
//    	pDrive = SmartDashboard.getNumber("P",0);
//    	iDrive = SmartDashboard.getNumber("I",0);
//    	dDrive = SmartDashboard.getNumber("D",0);
//    	target = SmartDashboard.getNumber("targetVelocity",0);
//    	pidDrive = new PID(dDrive, iDrive, dDrive, target, 0);
    	
    	
    	leftEncoder = new Encoder(1, 0, false, Encoder.EncodingType.k4X);
    	rightEncoder = new Encoder(2, 3, false, Encoder.EncodingType.k4X);
    	leftEncoder.setDistancePerPulse(5.03/256);
    	
    	pDrive = SmartDashboard.getNumber("P",0);
    	iDrive =  SmartDashboard.getNumber("I",0);
    	dDrive =  SmartDashboard.getNumber("D",0);
    	driveController = new PIDController(pDrive, iDrive, dDrive, leftEncoder, leftDrive1);
    	driveController.setSetpoint(10000.0);
    	driveController.setOutputRange(-1, 1);
    	
//    	leftEncoder.setMaxPeriod(.1);
//    	leftEncoder.setMinRate(10);
//    	leftEncoder.setDistancePerPulse(DIST_PER_PULSE);
//    	leftEncoder.setReverseDirection(false);
//    	leftEncoder.setSamplesToAverage(7);
//    	rightEncoder.setMaxPeriod(.1);
//    	rightEncoder.setMinRate(10);
//    	rightEncoder.setDistancePerPulse(DIST_PER_PULSE);
//    	rightEncoder.setReverseDirection(true);
//    	rightEncoder.setSamplesToAverage(7);
    }
    
    public void autonomousInit() {
    	leftEncoder.reset();
    }

    public void autonomousPeriodic() {
    	if (leftEncoder.getDistance() < 12.57) {
    		leftDrive1.set(.25);
    		leftDrive2.set(.25);
        	leftDrive3.set(.25);
        	rightDrive1.set(.25);
        	rightDrive2.set(.25);
        	rightDrive3.set(.25);
    	}
    	else {
    		leftDrive1.set(0);
    		leftDrive2.set(0);
        	leftDrive3.set(0);
        	rightDrive1.set(0);
        	rightDrive2.set(0);
        	rightDrive3.set(0);
    	}
    	System.out.println("Encoder Distance: " + leftEncoder.getDistance());
//    	timeX++;
//    	if (timeX == 1) {
//    		driveController.enable();
//    	}  	
//    	
//    	leftDrive2.set(driveController.get());
//    	leftDrive3.set(driveController.get());
//    	rightDrive1.set(driveController.get());
//    	rightDrive2.set(driveController.get());
//    	rightDrive3.set(driveController.get());
//    	
//    	System.out.println("\nRUN TIME #" + timeX + ":");
//    	System.out.println("Error: " + driveController.getError());
//    	System.out.println("Current PID Result: " + driveController.get());
//    	System.out.println("Encoder Value: " + leftEncoder.getDistance());
//    
//    }
//
//    public void teleopPeriodic() {
//    	drivetrain.arcadeDrive(-xbox1.getRawAxis(1), -xbox1.getRawAxis(4));
//    	drivetrain2.arcadeDrive(-xbox1.getRawAxis(1), -xbox1.getRawAxis(4));
//    	if(xbox1.getRawButton(5)){
//    		high=false;
//    	}
//    	
//    	if(xbox1.getRawButton(6)){
//    		high=true;
//    	}
//    	
//    	driveshifter.set(high);
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
}