/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4501.robot;

import java.util.Arrays;

import org.usfirst.frc.team4501.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	// SANDY THE SENSOR BOT HAS MY CODE NOT MADE FOR IT ON THE RIO. YOU NEED TO
	// REUPLOAD THIS SENSOR BOT CODE ONTO IT SORRY SORRY SORRY SORRY SORRY SORRY
	// SORRY SORRY SORRY SORRY SORRY SORRY SORRY SORRY SORRY SORRY SORRY SORRY
	// SORRY SORRY SORRY SORRY SORRY SORRY SORRY SORRY

	class Kontours implements Comparable<Kontours> {
		public double area;
		public double width;
		public double x;
		public double y;

		public Kontours(double area, double width, double x, double y) {
			this.area = area;
			this.width = width;
			this.x = x;
			this.y = y;
		}

		@Override
		public int compareTo(Kontours other) {
			// Descending Order
			return (int) Math.signum(other.area - area);
		}
	}

	class VisionSteering extends PIDSubsystem {
		public VisionSteering(double p, double i, double d) {
			super("VisionSteering", p, i, d);
			getPIDController().setContinuous(false);
			LiveWindow.addActuator("VisionSteering", "pid", getPIDController());
		}

		@Override
		protected void initDefaultCommand() {
		}

		@Override
		protected void usePIDOutput(double steeringOutput) {

			if (isEnabled() && (isAutonomous() || isTest())) {
				System.out.printf("%.2f Steering = %.1f, Drive = %.1f, Width = %.1f, Goal = %.1f\n",
						System.currentTimeMillis() / 1000., steeringOutput, pidDriveOutput, centerWidth, centerX);
				steeringOutput = Math.min(steeringOutput, 0.7);
				steeringOutput = Math.max(steeringOutput, -0.7);
				drive.arcadeDrive(pidDriveOutput, 0);//-steeringOutput);
				
				/*
				 * alternating between forward and rotating if ((period & 1) ==
				 * 0) { drive.tankDrive(pidDriveOutput, steeringOutput); }
				 */
			}
		}

		@Override
		protected double returnPIDInput() {
			// Calculate the distance between the center of the screen and the
			// center of the target
			if (!isAutonomous() && !isTest()) {
				return 0;
			}
			double deltaX = centerX - cameraCenterX;
			deltaX /= cameraWidth / 2;
			return deltaX;
		}
	}

	class VisionDrive extends PIDSubsystem {
		
		boolean reachedTargetWidth = false;
		
		
		public VisionDrive(double p, double i, double d) {
			super("VisionDrive", p, i, d);
			this.setSetpoint(visionDriveTargetWidth);
			getPIDController().setContinuous(false);
			LiveWindow.addActuator("VisionDrive", "pid", getPIDController());
		}

		@Override
		protected double returnPIDInput() {
			if (!isAutonomous() && !isTest()) {
				return 0;
			}
			if (centerWidth > visionDriveTargetWidth) {
				reachedTargetWidth = true;
			}
			
			return centerWidth;
		}

		@Override
		protected void usePIDOutput(double output) {
			// TODO Auto-generated method stub
			
			if (reachedTargetWidth) {
				pidDriveOutput = 0;
				return;
			}
			
			pidDriveOutput = output;
			
			if (pidDriveOutput >= 0.8) {
				pidDriveOutput = 0.8;
			}
			
			if (pidDriveOutput <= 0) {
				pidDriveOutput = 0;
			}
			// alternating between forward and rotation
			/*
			 * if ((period & 1) == 1) { drive.tankDrive(output, output); }
			 */

		}

		@Override
		protected void initDefaultCommand() {
			// TODO Auto-generated method stub

		}

		@Override
		public void disable() {
			// TODO Auto-generated method stub
			super.disable();
			pidDriveOutput = 0;
		}

		@Override
		public void enable() {
			// TODO Auto-generated method stub
			super.enable();
		}
	}

	public static double steeringKp = 1.4;
	public static double steeringKi = .06;
	public static double steeringKd = 1.0;
	public static double driveKp = .03;
	public static double driveKi = 0;
	public static double driveKd = .037;
	public static double visionDriveTargetWidth = 90;
	public static double cameraWidth = 320;
	public static double cameraHeight = 240;
	public static double cameraCenterX = cameraWidth / 2.0;
	public static double cameraCenterY = cameraHeight / 2.0;

	public static OI oi;
	public static Robot instance;
	public static final DriveTrain drive = new DriveTrain();

	VisionSteering visionSteering;
	VisionDrive visionDrive;
	Command autonomousCommand;

	public double centerY;
	public double centerX;
	public double centerWidth;
	public double pidDriveOutput;

	int period;

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
		visionSteering = new VisionSteering(steeringKp, steeringKi, steeringKd);
		visionDrive = new VisionDrive(driveKp, driveKi, driveKd);
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

	@Override
	public void testInit() {
		// TODO Auto-generated method stub
		super.testInit();
		visionDrive.enable();
		visionSteering.enable();
		period = 0;
	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
		// SmartDashboard.putData("Center Y[0]", );
		// visionSystem.enable();
		LiveWindow.run();
		getCenters();
		++period;
	}

	public boolean getCenters() {
		double[] tableX = netTable.getNumberArray("centerX", defaultValues);
		double[] tableY = netTable.getNumberArray("centerY", defaultValues);
		double[] tableWidth = netTable.getNumberArray("width", defaultValues);
		int count = Math.min(tableWidth.length, Math.min(tableX.length, tableY.length));
		
		if (count == 0) {
			return false;
		}

		Kontours[] countours = new Kontours[count];
		for (int i = 0; i < count; i++) {
			double area = tableX[i] * tableY[i];
			countours[i] = new Kontours(area, tableWidth[i], tableX[i], tableY[i]);
		}

		// TODO: Check to see that the two largest areas are aprox. equal
		Arrays.sort(countours);
		int targetIndex = 0;
		if (countours.length > 1) {
			if (countours[0].x < countours[1].x) {
					targetIndex = 1;
			}
		}
		
		centerX = countours[targetIndex].x;
		centerWidth = countours[targetIndex].width;
		
		return true;
	}
}





