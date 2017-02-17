package org.usfirst.frc.team4501.robot.subsystems;

import org.usfirst.frc.team4501.robot.RobotMap;
import org.usfirst.frc.team4501.robot.commands.StopLift;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Lift extends Subsystem {

	Talon liftTalon;
	
	public Lift(){
		liftTalon = new Talon(RobotMap.LIFT_TALON);
	}
	
	public void liftRobotUp() {
		liftTalon.set(-.75);
	}

	public void lowerRobotDown() {
		liftTalon.set(.75);
	}
	
	public void stopLift() {
		liftTalon.set(0);
	}
    public void initDefaultCommand() {
    	setDefaultCommand(new StopLift());
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

