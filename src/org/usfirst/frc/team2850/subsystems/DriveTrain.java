package org.usfirst.frc.team2850.subsystems;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PIDController;
import org.usfirst.frc.team2850.robot.*;

public class DriveTrain extends IterativeRobot{
	PIDController driveControllerLeft;
	PIDController driveControllerRight;
	PIDController gyroController;
	
	public double pDrive;
	public double iDrive;
	public double dDrive;
	public double pGyro;
	public double iGyro;
	public double dGyro;
	
	public void pidInit() {
//		pDrive = SmartDashboard.getNumber("P",0);
//		iDrive =  SmartDashboard.getNumber("I",0);
//		dDrive =  SmartDashboard.getNumber("D",0);
		pDrive = .07;
		iDrive =  0;
		dDrive =  0.007;
		pGyro = 0.01;
		iGyro = 0;
		dGyro = 0.001;
		
		driveControllerLeft = new PIDController(pDrive, iDrive, dDrive, Robot.leftEncoder, Robot.leftDrive1);
		driveControllerLeft.setOutputRange(-1, 1);
		driveControllerRight = new PIDController(pDrive, iDrive, dDrive, Robot.rightEncoder, Robot.rightDrive1);
		driveControllerRight.setOutputRange(-1, 1);
		gyroController = new PIDController(pGyro, iGyro, dGyro, Robot.gyro, Robot.leftDrive1);
		driveControllerLeft.setOutputRange(-1, 1);
	}
	
	public void pidDrive(double distance){
		driveControllerLeft.enable();
		driveControllerRight.enable();
		
		driveControllerLeft.setSetpoint(distance);
		driveControllerRight.setSetpoint(-distance);
		if (driveControllerLeft.getError() > 2 && driveControllerRight.getError() > 2) {
			Robot.leftDrive2.set(driveControllerLeft.get());
			Robot.leftDrive3.set(driveControllerLeft.get());
			Robot.rightDrive2.set(driveControllerRight.get());
			Robot.rightDrive3.set(driveControllerRight.get());
		}
		else {
			Robot.leftDrive2.set(0);
			Robot.leftDrive3.set(0);
			Robot.rightDrive2.set(0);
			Robot.rightDrive3.set(0);
			driveControllerLeft.disable();
			driveControllerRight.disable();
		}
	}
	
	public void pidGyro(double angle) {
		gyroController.enable();
		
		gyroController.setSetpoint(angle);
		
		if (gyroController.getError() > 2) {
			Robot.leftDrive2.set(gyroController.get());
			Robot.leftDrive3.set(gyroController.get());
			Robot.rightDrive2.set(gyroController.get());
			Robot.rightDrive3.set(gyroController.get());
			System.out.println("Gyro Reading: " + Robot.gyro.getAngle());
		}
		else {
			Robot.leftDrive2.set(0);
			Robot.leftDrive3.set(0);
			Robot.rightDrive2.set(0);
			Robot.rightDrive3.set(0);
			gyroController.disable();
		}
	}
	
}
