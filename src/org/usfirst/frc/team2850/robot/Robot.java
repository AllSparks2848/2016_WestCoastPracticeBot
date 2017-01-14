
package org.usfirst.frc.team2850.robot;

import org.usfirst.frc.team2850.subsystems.Shooter;
import org.usfirst.frc.team2850.util.PID;

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
	
	public static Spark shooterMotor;                //added V1.0
	public static Encoder shooterEncoder;
	public static Shooter shooter;
	public double shooterCurrent = 0;
	public double shooterP = .005;
	public double shooterI = .005;
	public double shooterD = .005;
	public double target = 10;
	public PID driveTrainPID;
	
	
	
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
    	
    	leftEncoder = new Encoder(2,3,false, Encoder.EncodingType.k4X);
    	rightEncoder = new Encoder(4,5,false, Encoder.EncodingType.k4X);
    	
    	
    	drivetrain= new RobotDrive(leftDrive1, leftDrive2, rightDrive1, rightDrive2);
    	drivetrain2= new RobotDrive(leftDrive3, rightDrive3);
    	
    	compressor = new Compressor();
    	driveshifter=new Solenoid(0);
    	driveTrainPID = new PID(shooterP,shooterI,shooterD,target,0);
    	
       
//    	shooterMotor = new Spark(6);
//    	shooterEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X); //where are Channels A and B plugged in?
//    	shooter = new Shooter(shooterMotor, shooterEncoder);
//    	
    	
//    	leftEncoder = new Encoder(, , false, Encoder.EncodingType.k4X);
//    	rightEncoder = new Encoder(, , false, Encoder.EncodingType.k4X);
    	
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
    	 
    	
    }

    public void autonomousPeriodic() {
    	leftDrive1.set(driveTrainPID.compute(leftEncoder.getRate()));
    	leftDrive2.set(driveTrainPID.compute(leftEncoder.getRate()));
    	leftDrive3.set(driveTrainPID.compute(leftEncoder.getRate()));
    	rightDrive1.set(driveTrainPID.compute(rightEncoder.getRate()));
    	rightDrive2.set(driveTrainPID.compute(rightEncoder.getRate()));
    	rightDrive3.set(driveTrainPID.compute(rightEncoder.getRate()));
    	
    	
    	if(driveTrainPID.onTarget()){
    		rightDrive1.set(0);
    		rightDrive2.set(0);
    		rightDrive3.set(0);
    		leftDrive1.set(0);
    		leftDrive2.set(0);
    		leftDrive3.set(0);
    		
    	}
    	
    }

    public void teleopPeriodic() {
    	drivetrain.arcadeDrive(-xbox1.getRawAxis(1), -xbox1.getRawAxis(4));
    	drivetrain2.arcadeDrive(-xbox1.getRawAxis(1), -xbox1.getRawAxis(4));
    	if(xbox1.getRawButton(5)){
    		high=false;
    	}
    	
    	if(xbox1.getRawButton(6)){
    		high=true;
    	}
    	
    	driveshifter.set(high);
    	
    	
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
}