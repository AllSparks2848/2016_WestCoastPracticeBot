package org.usfirst.frc.team2850.subsystems;

import org.usfirst.frc.team2850.robot.Robot;
import org.usfirst.frc.team2850.util.PID;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter {
	private static Spark shooterMotor;
	private static Encoder shooterEncoder;
	public static PID shooterpid;
	
	Robot robot = new Robot();
	double p = SmartDashboard.getNumber("P", 0.004);
	double i = SmartDashboard.getNumber("I", 0.002);
	double d = SmartDashboard.getNumber("D", 0.0);
	double target = SmartDashboard.getNumber("targetVelocity", 60.0); //feet per sec
	
	
	public double instantVelocity;
	
	public Shooter(Spark shooterMotor, Encoder shooterEncoder)
	{
		Shooter.shooterMotor = shooterMotor;
		Shooter.shooterEncoder = shooterEncoder;
		
		shooterEncoder.setDistancePerPulse(4*3.142/3); //775 at 3:1 (encoder after gearing), 4069 encoder ticks, 2in wheel
	}
	public void shoot()
	{
		shooterMotor.set(1.0);
		instantVelocity = shooterEncoder.getRate();
	}
	public void shootPID()
	{
		shooterMotor.set(shooterpid.compute(shooterEncoder.getRate()));
		instantVelocity = shooterEncoder.getRate();
	}
}

//if(xbox1.getRawButton(0))
//	motor1.set(1.0);
//if(maxCurrent<pdp.getCurrent(0))
//	maxCurrent=pdp.getCurrent(0);
//sumCurrent += pdp.getCurrent(0);
//avgCurrent = sumCurrent/iterations;
//iterations++;
//System.out.println("Max Current Draw: " + Double.toString(maxCurrent) + "/n Avg Current Draw: " + Double.toString(avgCurrent));