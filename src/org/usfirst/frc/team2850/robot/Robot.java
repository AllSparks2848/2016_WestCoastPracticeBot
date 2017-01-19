
package org.usfirst.frc.team2850.robot;

import org.usfirst.frc.team2850.subsystems.Shooter;

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
	
	public static ADXRS450_Gyro gyro;
	
	public static int timeX;
	
	PIDController driveControllerLeft;
	PIDController driveControllerRight;
	PIDController gyroController;
	
    public static boolean high;
    
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
    	
    	gyro = new ADXRS450_Gyro(); 
    	
    	timeX = 0;
    	
    	leftEncoder = new Encoder(1, 0, false, Encoder.EncodingType.k4X);
    	rightEncoder = new Encoder(3, 2, false, Encoder.EncodingType.k4X);
    	
    	leftEncoder.setDistancePerPulse(0.0115);
    	rightEncoder.setDistancePerPulse(-0.0117);
    	
//    	pDrive = SmartDashboard.getNumber("P",0);
//    	iDrive =  SmartDashboard.getNumber("I",0);
//    	dDrive =  SmartDashboard.getNumber("D",0);
    	
    	pDrive = .01;
    	iDrive =  0;
    	dDrive =  0.001;
    	driveControllerLeft = new PIDController(pDrive, iDrive, dDrive, leftEncoder, leftDrive1);
    	driveControllerLeft.setSetpoint(0);
    	driveControllerLeft.setOutputRange(-1, 1);
    	driveControllerRight = new PIDController(pDrive, iDrive, dDrive, rightEncoder, rightDrive1);
    	driveControllerRight.setSetpoint(0);
    	driveControllerRight.setOutputRange(-1, 1);
    	
    	pGyro = 0.01;
    	iGyro = 0;
    	dGyro = 0;
    	gyroController = new PIDController(pGyro, iGyro, dGyro, gyro, leftDrive1);
    	gyroController.setSetpoint(90);
    	driveControllerLeft.setOutputRange(-1, 1);
    	gyro.calibrate();
    }
    
    public void autonomousInit() {
    	leftEncoder.reset();
    	rightEncoder.reset();
    	timeX = 0;
    	gyro.calibrate();
    }

    public void autonomousPeriodic() {
    	timeX++;
    	if (timeX == 1) {
    		driveControllerLeft.enable();
    		driveControllerRight.enable();
    		gyroController.enable();
    	}

    	leftDrive2.set(driveControllerLeft.get());
    	leftDrive3.set(driveControllerLeft.get());
    	rightDrive2.set(driveControllerRight.get());
    	rightDrive3.set(driveControllerRight.get());
    	
    	rightDrive1.set(gyroController.get());
    	rightDrive2.set(gyroController.get());
    	rightDrive3.set(gyroController.get());
    	leftDrive1.set(gyroController.get());
    	leftDrive2.set(gyroController.get());
    	leftDrive3.set(gyroController.get());
    	
    	
    	System.out.println("\nRUN TIME #" + timeX + ":");
    	System.out.println("Error (Left): " + driveControllerLeft.getError());
    	System.out.println("Error (Right): " + driveControllerRight.getError());
    	System.out.println("Current PID Result (Left): " + driveControllerLeft.get());
    	System.out.println("Current PID Result (Right): " + driveControllerRight.get());
    	System.out.println("Encoder Value (Left): " + leftEncoder.getDistance());
    	System.out.println("Encoder Value (Right): " + rightEncoder.getDistance());
    	System.out.println("Gyro Angle: " + gyro.getAngle());
    	System.out.println("Gyro Rate: " + gyro.getRate());
    	System.out.println("Gyro PID Output: " + gyroPower);
    	System.out.println("Right Drive 1 Power: " + rightDrive1.get());
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
    	
    }
    
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
}