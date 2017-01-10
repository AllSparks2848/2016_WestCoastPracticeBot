package org.usfirst.frc.team2850.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;

public class Shooter {
	private static Spark shooterMotor;
	private static Encoder shooterEncoder;
	
	Robot robot =new Robot();
	double p = .004;
	double i = .002;
	double d = .000;
	double target = 10; //in feet
	PID shooterpid = new PID(p,i,d, target, 0);
	
	public double instantVelocity;
	
	public Shooter(Spark shooterMotor, Encoder shooterEncoder)
	{
		Shooter.shooterMotor = shooterMotor;
		Shooter.shooterEncoder = shooterEncoder;
		
		shooterEncoder.setDistancePerPulse(.2); //encoder ticks, gearing to get fps tangential speed
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