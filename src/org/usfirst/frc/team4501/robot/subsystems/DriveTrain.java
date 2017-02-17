package org.usfirst.frc.team4501.robot.subsystems;

import org.usfirst.frc.team4501.robot.Robot;
import org.usfirst.frc.team4501.robot.RobotMap;
import org.usfirst.frc.team4501.robot.commands.Drive;
import org.usfirst.frc.team4501.robot.commands.EncRate;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveTrain extends Subsystem {

	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	DriveTrain drive;

	Talon leftTalon;
	Talon rightTalon;

	RobotDrive driveShit;
	
	Encoder encoder;

	public DriveTrain() {
		leftTalon = new Talon(RobotMap.LEFT_TALON);
		rightTalon = new Talon(RobotMap.RIGHT_TALON);

		driveShit = new RobotDrive(leftTalon, rightTalon);
		
		encoder = new Encoder(RobotMap.ENCODER, RobotMap.ENCODER2, false, Encoder.EncodingType.k4X);
	}

	public void tankDrive(double leftSpin, double rightSpin) {
		driveShit.tankDrive(leftSpin, rightSpin);
	}

	public void arcadeDrive(double forward, double rotate) {
		driveShit.arcadeDrive(forward, rotate);
	}

	public void autoMove() {
		leftTalon.set(1);
		rightTalon.set(1);
	}
	
	public void encRate(){
		double encRate = encoder.getRate();
		SmartDashboard.putDouble("Encoder Rate", encRate);
		SmartDashboard.putDouble("Encoder angle", encoder.getDistance());
		System.out.println(encRate);
	}
	

	public void initDefaultCommand() {
		setDefaultCommand(new Drive());
	}
}
