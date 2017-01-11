package org.usfirst.frc.team2850.util;

public class PID {
	
	private double target;
	
	private double pterm;
	private double iterm;
	private double dterm;
	
	private long nanotime;
	private long lastnanotime;
	
	private double i = 0;
	private double min = -100;
	private double max = 100;
	private double mini = -100;
	private double maxi = 100;
	private double lastinput = 0;
	
	public PID(double pterm, double iterm, double dterm, double target, double startingval) {
		this.pterm = pterm; //initializes parameters
		this.iterm = iterm;
		this.dterm = dterm;
		this.target = target;
		
		nanotime = System.nanoTime(); // initializes time
		lastnanotime = System.nanoTime();
		
		lastinput = startingval; // gives a starting value to prevent big starting jump
	}
	
	public void setTarget(double target) { // sets target value
		this.target = target;
	}
	
	public double getTarget() {
		return target;
	}
	
	public double compute(double input) {
		double error = target-input; //calculate error of the system	
		double p = pterm*error; //get proportional term of error
		
		nanotime = System.nanoTime();
		double deltatime = (nanotime-lastnanotime)/1000000000.0; //calculate delta time in nano seconds and divide by a billion to get seconds
		lastnanotime = nanotime;
		
		double d = dterm*(input-lastinput)/deltatime; //calculate derivative of error, preventing derivative kick by only looking at input
		lastinput = input;
		
		i += iterm*error*deltatime; // calculates integral of error
		if(i < mini) i = mini; //clamps to prevent integral windup
		if(i > maxi) i = maxi;
		
		double output = p+i-d;
		if(output < min) output = min; // clamps output
		if(output > max) output = max;
		
		return output;
	}
	
	public void setTuning(double pterm, double iterm, double dterm) { // sets p, i, and d terms
		this.pterm = pterm;
		this.iterm = iterm;
		this.dterm = dterm;
	}
	
	public void setBounds(double min, double max) { // sets bounds for overall output
		this.min = min;
		this.max = max;
	}
	
	public void setITermBounds(double mini, double maxi) { // sets bounds for integral windup clamp
		this.mini = mini;
		this.maxi = maxi;
	}

}