package org.usfirst.frc.team4501.robot.subsystems;

import org.usfirst.frc.team4501.robot.OI;
import org.usfirst.frc.team4501.robot.Robot;
import org.usfirst.frc.team4501.robot.RobotMap;
import org.usfirst.frc.team4501.robot.commands.Drive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class DriveTrain extends Subsystem {
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	DriveTrain drive;
	
	Talon talon1;
	Talon talon2;
	Talon talon3;
	
	RobotDrive driveShit;
	
	
	public DriveTrain(){
		talon1 = new Talon(RobotMap.TALON);
		talon2 = new Talon(RobotMap.TALON2);
		talon3 = new Talon(RobotMap.TALON3);
				
		
		driveShit = new RobotDrive(talon1, talon2);
	}
	
	public void move(double xAxis, double yAxis){
			driveShit.tankDrive(xAxis, yAxis);
	}
	
	public void autoMove(){
		talon1.set(1);
		talon2.set(1);
	}
	
	public void autoOpen(){
		talon3.set(1);
	}
	
	public void openClaw(double talon3){
		
		this.talon3.set(talon3);
		
	}
	
	public void closeClaw(double talon3){
		this.talon3.set(talon3);
	}

    public void initDefaultCommand() {
    	setDefaultCommand(new Drive());
    }
}

