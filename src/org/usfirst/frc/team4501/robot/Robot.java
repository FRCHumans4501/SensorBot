/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4501.robot;

import java.util.Arrays;

import org.usfirst.frc.team4501.robot.commands.EncRate;
import org.usfirst.frc.team4501.robot.subsystems.DriveTrain;
import org.usfirst.frc.team4501.robot.subsystems.Lift;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
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
	enum VisionMode {
		ROTATE, MOVE, DONE, DISABLED;
	}

	private VisionMode visionMode = VisionMode.DISABLED;

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

	class VisionRotate extends PIDSubsystem {
		public VisionRotate(double p, double i, double d) {
			super("VisionRotate", p, i, d);
			getPIDController().setContinuous(false);
			getPIDController().setOutputRange(-maxRotateSpeed, maxRotateSpeed);
			setSetpoint(cameraCenterX);
			LiveWindow.addActuator("VisionRotate", "pid", getPIDController());
		}

		@Override
		public void enable() {
			super.enable();
			targetX = 0;
		}

		@Override
		protected void initDefaultCommand() {
		}

		@Override
		protected double returnPIDInput() {
			rotateAvgError = getPIDController().getAvgError();

			// Calculate the distance between the center of the screen and the
			// center of the target
			if (isAutonomous() || isTest()) {
				targetX = centerX;
			}
			return targetX;
		}

		@Override
		protected void usePIDOutput(double output) {
			switch (visionMode) {

			case ROTATE:
				if (period > 10 && Math.abs(rotateAvgError) < maxRotateError) {
					visionMode = VisionMode.MOVE;
					pidRotateOutput = 0;
				} else {
					pidRotateOutput = output;
				}
				break;

			default:
				pidRotateOutput = 0;
				break;
			}

		}

	}

	class VisionMove extends PIDSubsystem {
		public VisionMove(double p, double i, double d) {
			super("VisionMove", p, i, d);
			this.setSetpoint(visionMoveTargetWidth);
			getPIDController().setContinuous(false);
			getPIDController().setOutputRange(0, maxMoveSpeed);
			LiveWindow.addActuator("VisionMove", "pid", getPIDController());
		}

		@Override
		protected double returnPIDInput() {

			switch (visionMode) {

			case MOVE:
				if (centerWidth > visionMoveTargetWidth) {
					visionMode = VisionMode.DONE;
				} else if ((Math.abs(rotateAvgError) > maxRotateErrorDurringMove)) {
					visionRotate.getPIDController().setOutputRange(-maxAdjustedRotateSpeed, maxAdjustedRotateSpeed);
					visionMode = VisionMode.ROTATE;
				}
				break;
			default:
				break;
			}

			return centerWidth;
		}

		@Override
		protected void usePIDOutput(double output) {
			switch (visionMode) {

			case MOVE:
				pidMoveOutput = output;
				break;

			default:
				pidMoveOutput = 0;
				break;
			}
		}

		@Override
		protected void initDefaultCommand() {
		}

		@Override
		public void disable() {
			super.disable();
			pidMoveOutput = 0;
		}

		@Override
		public void enable() {
			super.enable();
			visionMode = VisionMode.ROTATE;
		}
	}

	public static double rotateKp = 0.4;
	public static double rotateKi = .2;
	public static double rotateKd = 1.5;
	public static double maxRotateSpeed = .6;
	public static double maxAdjustedRotateSpeed = .4;
	public static double moveKp = 0.5;
	public static double moveKi = .02;
	public static double moveKd = 0.3;
	public static double maxMoveSpeed = .6;
	public static double visionMoveTargetWidth = 60;
	public static double maxRotateError = 15;
	public static double maxRotateErrorDurringMove = 40;
	public static double cameraWidth = 320;
	public static double cameraHeight = 240;
	public static double cameraCenterX = cameraWidth / 2.0;
	public static double cameraCenterY = cameraHeight / 2.0;
	public static double maxDeltaChange = .5;

	public static OI oi;
	public static Robot instance;
	public static final DriveTrain drive = new DriveTrain();
	public static final Lift lift = new Lift();

	VisionRotate visionRotate;
	VisionMove visionMove;
	Command autonomousCommand;

	public double centerY;
	public double centerX;
	public double centerWidth;

	private double targetX;
	private double rotateAvgError;
	public double pidMoveOutput;
	public double pidRotateOutput;

	int period;

	public double[] defaultValues = new double[4];

	public NetworkTable netTable;
	
	SmartDashboard dashboard;

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
		visionRotate = new VisionRotate(rotateKp, rotateKi, rotateKd);
		visionMove = new VisionMove(moveKp, moveKi, moveKd);
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
		super.testInit();
		visionMode = VisionMode.ROTATE;
		period = 0;
		visionRotate.enable();
		visionMove.enable();
		centerWidth = Double.MIN_VALUE;
		getCenters();
	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
		LiveWindow.run();
		getCenters();
		++period;

		switch (visionMode) {
		case ROTATE:
		case MOVE:
			drive.arcadeDrive(-pidMoveOutput, pidRotateOutput);
			break;

		default:
			drive.arcadeDrive(0, 0);
			break;
		}

		System.out.printf("%.2f Mode=%s Rotate=%.1f Move=%.1f targetX=%.1f Width=%.1f rotateAvgErr=%.1f\n",
				System.currentTimeMillis() / 1000., visionMode, pidRotateOutput, pidMoveOutput, targetX, centerWidth,
				rotateAvgError);
	}

	@Override
	public void disabledPeriodic() {
		super.disabledPeriodic();
		visionMode = VisionMode.DISABLED;
	}

	public boolean getCenters() {
		double[] tableX = netTable.getNumberArray("centerX", defaultValues);
		double[] tableY = netTable.getNumberArray("centerY", defaultValues);
		double[] tableWidth = netTable.getNumberArray("width", defaultValues);
		double[] tableArea = netTable.getNumberArray("area", defaultValues);

		// NetworkTables aren't updated atomically and therefore the lengths can
		// differ.
		int count = Math.min(tableX.length, tableY.length);
		count = Math.min(count, tableWidth.length);
		count = Math.min(count, tableArea.length);

		if (count == 0) {
			return false;
		}

		Kontours[] kontours = new Kontours[count];
		for (int i = 0; i < count; i++) {
			double area = tableArea[i];
			kontours[i] = new Kontours(area, tableWidth[i], tableX[i], tableY[i]);
		}

		// Between the two with the largest areas, use the rightmost contour.
		Arrays.sort(kontours);
		int targetIndex = 0;
		if (kontours.length > 1) {
			if (kontours[0].x < kontours[1].x) {
				targetIndex = 1;
			}
		}
		double newWidth = kontours[targetIndex].width;
		if (centerWidth == Double.MIN_VALUE){
			centerWidth = newWidth;
		}
//		double deltaChange = Math.abs((centerWidth-newWidth)/centerWidth);
//		if (deltaChange > maxDeltaChange) {
//			System.out.println("Invalid Width: " + newWidth);
//			return false;
//		}
		
		centerX = kontours[targetIndex].x;
		centerWidth = newWidth;
		

		return true;
	}
}
