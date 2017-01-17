package org.usfirst.frc.team4501.robot;

public class PID {
	double lastTime;
	double Setpoint;
	double errSum, lastErr;
	double kp, ki, kd;
	
	public PID(double Kp, double Ki, double Kd)
	{
	   kp = Kp;
	   ki = Ki;
	   kd = Kd;
	}
	
	public double Compute(double input)
	{
	   /*How long since we last calculated*/
	   double now = System.currentTimeMillis()/1000.0;
	   double timeChange = (now - lastTime);
	  
	   /*Compute all the working error variables*/
	   double error = Setpoint - input;
	   errSum += (error * timeChange);
	   double dErr = (error - lastErr) / timeChange;
	  
	   /*Compute PID Output*/
	   double Output = kp * error + ki * errSum + kd * dErr;
	  
	   /*Remember some variables for next time*/
	   lastErr = error;
	   lastTime = now;
	   
	   return Output;
	}
	  
	

}
