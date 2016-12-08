package org.usfirst.frc.team4501.robot.subsystems;

import org.usfirst.frc.team4501.robot.OI;
import org.usfirst.frc.team4501.robot.Robot;
import org.usfirst.frc.team4501.robot.RobotMap;
import org.usfirst.frc.team4501.robot.commands.Drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI.Port;
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
	public AHRS navx;

	Talon leftTalon;
	Talon rightTalon;
	Talon ClawTalon;

	RobotDrive driveShit;
	
	private static final double turnP = 1.0 / 15;
	private static final double turnI = 0;
	private static final double turnD = 0;
	public PIDController accuracy;

	public DriveTrain() {
		leftTalon = new Talon(RobotMap.TALON_LEFT);
		rightTalon = new Talon(RobotMap.TALON_RIGHT);
		ClawTalon = new Talon(RobotMap.TALON_CLAW);
		driveShit = new RobotDrive(leftTalon, rightTalon);

		try {
			navx = new AHRS(Port.kMXP);
		} catch (RuntimeException ex) {
			DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
		}

		this.navx.setPIDSourceType(PIDSourceType.kDisplacement);
		accuracy = new PIDController(turnP, turnI, turnD, this.navx, new PIDOutput() {
			@Override
			public void pidWrite(double arg0) {
				// Do Nothing
			}
		});
		this.accuracy.setInputRange(-180, 180);
		this.accuracy.setContinuous(true);
		this.accuracy.setOutputRange(-0.8,  0.8);
	}

	public void initDefaultCommand() {
		setDefaultCommand(new Drive());
	}

	public void move(double xAxis, double yAxis) {
		driveShit.arcadeDrive(xAxis, yAxis);
	}

	public void autoMove() {
		leftTalon.set(1);
		rightTalon.set(1);
	}

	public void autoOpen() {
		ClawTalon.set(1);
	}
	
	public double getGyroAngle() {
		return navx.getAngle();
	}
	
	public double getGyroRate() {
		return navx.getRate();
	}

	public void openClaw(double talon3) {
		this.ClawTalon.set(talon3);
	}

	public void closeClaw(double talon3) {
		this.ClawTalon.set(talon3);
	}
	
	public void gyroReset() {
		this.navx.reset();
	}
	
	public void getData() {
    	SmartDashboard.putNumber("Gyro Angle", this.getGyroAngle());
    	SmartDashboard.putNumber("Gyro Rate", this.getGyroRate());
	}
}
