
package org.usfirst.frc.team2850.robot;

import org.usfirst.frc.team2850.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.SensorBase;
import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
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
	public static Spark shooter;
	public static Encoder shooterEncoder;
	public static Compressor compressor;
	public static Solenoid driveshifter;
	
	public static Encoder leftEncoder;
	public static Encoder rightEncoder;
	
	public static double pDrive;
	public static double iDrive;
	public static double dDrive;
	public static double pGyro;
	public static double iGyro;
	public static double dGyro;
	
	public static double shooterPID = 0;
	public static double avgRate = 0;
	public static int n = 0;
	
	public static ADXRS450_Gyro gyro;
	
	public static int timeX;
	
    public static boolean high;
    
    DriveTrain driveBase = new DriveTrain();
    
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
    	shooter = new Spark(7);
    	
    	shooterEncoder = new Encoder(4, 5, false, Encoder.EncodingType.k4X);
    	
    	drivetrain= new RobotDrive(leftDrive1, leftDrive2, rightDrive1, rightDrive2);
    	drivetrain2= new RobotDrive(leftDrive3, rightDrive3);
    	
    	compressor = new Compressor();
    	driveshifter=new Solenoid(0);
    	
    	gyro = new ADXRS450_Gyro(); 
    	
    	timeX = 0;
    	
    	leftEncoder = new Encoder(1, 0, false, Encoder.EncodingType.k4X);
    	rightEncoder = new Encoder(3, 2, false, Encoder.EncodingType.k4X);
    	
    	leftEncoder.setDistancePerPulse(0.0115);
    	rightEncoder.setDistancePerPulse(-0.0117);
    	
    	gyro.calibrate();
    }
    
    public void autonomousInit() {
    	leftEncoder.reset();
    	rightEncoder.reset();
    	gyro.reset();
    	timeX = 0;
    	driveBase.pidInit();
    	driveBase.pidGyro(90);
    }

    public void autonomousPeriodic() {
    	timeX++;
    	
//    	leftDrive2.set(driveControllerLeft.get());
//    	leftDrive3.set(driveControllerLeft.get());
//    	rightDrive2.set(driveControllerRight.get());
//    	rightDrive3.set(driveControllerRight.get());
//    	
//    	rightDrive1.set(gyroController.get());
//    	rightDrive2.set(gyroController.get());
//    	rightDrive3.set(gyroController.get());
//    	leftDrive1.set(gyroController.get());
//    	leftDrive2.set(gyroController.get());
//    	leftDrive3.set(gyroController.get());
    }
    
    public void teleopPeriodic() {
    	drivetrain.arcadeDrive(-xbox1.getRawAxis(1), -xbox1.getRawAxis(4));
    	drivetrain2.arcadeDrive(-xbox1.getRawAxis(1), -xbox1.getRawAxis(4));
    	if(xbox1.getRawButton(5)){
    		high=false;
    	}
    	
    	if(xbox1.getRawButton(6)) {
    		high=true;
    	}
    	
    	driveshifter.set(high);
    	
    	shooterPID = (580-shooterEncoder.getRate())*.04;
    	if(shooterPID > 1)
    	{
    		shooterPID = 1;
    	}
    	if(shooterPID < 0)
    	{
    		shooterPID = 0;
    	}
    if(xbox1.getRawButton(2))
    {
    	shooter.set(shooterPID);
    }
    else
    	shooter.set(0);
    
    
    if(shooterPID == 0) {
    	System.out.println("Encoder PID Out: " + shooterPID);
    }
    //System.out.println("Encoder Rate: " + shooterEncoder.getRate());
    
    	}
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
}