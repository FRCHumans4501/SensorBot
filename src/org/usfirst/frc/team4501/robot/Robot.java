/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4501.robot;

import org.usfirst.frc.team4501.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	class VisionSystem extends PIDSubsystem {
		public VisionSystem(double p, double i, double d) {
			super("VisionSystem", p, i, d);
			getPIDController().setContinuous(false);
			LiveWindow.addActuator("VisionSystem", "pid", getPIDController());
			System.out.println("vision init :" + getSmartDashboardType());
		}

		@Override
		protected void initDefaultCommand() {
		}

		@Override
		protected void usePIDOutput(double output) {
			if (isEnabled() && (isAutonomous() || isTest())) {
				// drive.move(output, -output);
				System.out.printf("drive = %.1f, %.1f\n", output, -output);
			}
		}

		@Override
		protected double returnPIDInput() {
			// Calculate the distance between the center of the screen and the
			// center of the target
			if (!isAutonomous() && !isTest()) {
				return 0;
			}

			getCenters();
			double deltaX = centerXLeft - cameraCenterX;
			deltaX /= cameraWidth / 2;
			return deltaX;
		}
	}

	public static double Kp = 1;
	public static double Ki = 0;
	public static double Kd = 0;
	public static double cameraWidth = 320;
	public static double cameraHeight = 240;
	public static double cameraCenterX = cameraWidth / 2.0;
	public static double cameraCenterY = cameraHeight / 2.0;

	public static OI oi;
	public static Robot instance;
	public static final DriveTrain drive = new DriveTrain();

	VisionSystem visionSystem;

	Command autonomousCommand;

	public double centerYLeft;
	public double centerXLeft;

	public double[] defaultValues = new double[4];

	public NetworkTable netTable;

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
		visionSystem = new VisionSystem(Kp, Ki, Kd);
	}

	public void autonomousInit() {
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
		// SmartDashboard.putData("Center Y[0]", );
		visionSystem.enable();
		LiveWindow.run();
	}

	public void getCenters() {
		double[] tableX = netTable.getNumberArray("centerX", defaultValues);
		double[] tableY = netTable.getNumberArray("centerY", defaultValues);
		if (tableX.length > 0 && tableY.length > 0) {
			centerXLeft = tableX[0];
			centerYLeft = tableY[0];
		}
	}
}
