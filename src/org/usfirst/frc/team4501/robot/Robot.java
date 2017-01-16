/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4501.robot;


import org.usfirst.frc.team4501.robot.commands.TurnToCenter;
import org.usfirst.frc.team4501.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;




/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	public static OI oi;
	
    Command autonomousCommand;
    public static final DriveTrain drive = new DriveTrain();
    public NetworkTable netTable;
    public double centerY;
    public double centerX;
    public double[] defaultValues = new double[4];
    public static Robot instance;
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     * 
     *
     */
    
    public void robotInit() {
    	instance = this;
    	oi = new OI();
    	netTable = NetworkTable.getTable("GRIP/myContoursReport");
    }

    public void autonomousInit() {
    	autonomousCommand = new TurnToCenter();
        autonomousCommand.start();
    	System.out.println("Auto Init");
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    public void teleopInit() {
	// This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to 
        // continue until interrupted by another command, remove
        // this line or comment it out.
    	}

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	Scheduler.getInstance().run();
    	
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    	//SmartDashboard.putData("Center Y[0]", );
    	LiveWindow.run();
    }
    
    public void getCenters() {
       	double[] tableX = netTable.getNumberArray("centerX", defaultValues);
    	double[] tableY = netTable.getNumberArray("centerY", defaultValues);
    	if (tableX.length > 0 && tableY.length > 0){
    	centerX = tableX[0];
    	centerY = tableY[0];
    	}
    }
    
}
