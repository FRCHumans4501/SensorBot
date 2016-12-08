package org.usfirst.frc.team4501.robot.commands;

import org.usfirst.frc.team4501.robot.Robot;
import org.usfirst.frc.team4501.util.PIDTimer;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TurnWithGyro extends Command {

	private double degrees;
	private PIDTimer timer;
	
    public TurnWithGyro(double degrees) {
    	requires(Robot.drive);
    	this.degrees = degrees;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.drive.accuracy.setSetpoint(this.degrees);
    	Robot.drive.accuracy.enable();
    	this.timer = new PIDTimer(() -> Robot.drive.accuracy.get(), 0, 1, 300);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	this.timer.update();
    	Robot.drive.move(0, Robot.drive.accuracy.get());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return this.timer.isDone();
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.drive.move(0, 0);
    	Robot.drive.accuracy.disable();
    }
}
