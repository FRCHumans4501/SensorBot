package org.usfirst.frc.team4501.robot.commands;

import org.usfirst.frc.team4501.robot.Robot;


import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 *
 */
public class AutoMove extends CommandGroup {

    public AutoMove() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.drive);
    	
    	addSequential(new autoMover());
    	addSequential(new WaitCommand(1));
    	addSequential(new autoOpen());
    }

    
}
