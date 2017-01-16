package org.usfirst.frc.team4501.robot.commands;

import org.usfirst.frc.team4501.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TurnToCenter extends Command {
double vodCenterX = 320/2;
double vodCenterY = 240/2;
double margin = 40;


    public TurnToCenter() {
    	requires(Robot.drive);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.instance.getCenters();
    	if (Robot.instance.centerX < (vodCenterX - margin)) {
    		Robot.drive.move(.6, -.6);
    	} else if (Robot.instance.centerX > (vodCenterX + margin)) {
    		Robot.drive.move(-.6, .6);
    	} else {
    		Robot.drive.move(0, 0);
    	}
    	System.out.println("Center X = " + Robot.instance.centerX);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return !Robot.instance.isAutonomous();
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
