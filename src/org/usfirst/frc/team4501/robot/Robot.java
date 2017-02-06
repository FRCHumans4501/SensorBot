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

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	enum VisionMode {
		ROTATE, MOVE, DONE
	}

	private VisionMode visionMode = VisionMode.ROTATE;

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
			setSetpoint(cameraCenterX);
			LiveWindow.addActuator("VisionRotate", "pid", getPIDController());
		}

		@Override
		public void enable() {
			super.enable();
		}

		@Override
		protected void initDefaultCommand() {
		}

		@Override
		protected double returnPIDInput() {
			rotateAvgError = getPIDController().getAvgError();

			// Calculate the distance between the center of the screen and the
			// center of the target
			targetX = getSetpoint();
			if (isAutonomous() || isTest()) {
				targetX = centerX;
			}
			return targetX;
		}

		@Override
		protected void usePIDOutput(double output) {
			if (visionMode != VisionMode.ROTATE) {
				pidRotateOutput = 0;
				return;
			}

			if (rotateAvgError < maxRotateError) {
				visionMode = VisionMode.MOVE;
				pidRotateOutput = 0;
				return;
			}

			if (isEnabled() && (isAutonomous() || isTest())) {
				pidRotateOutput = output;
				if (pidRotateOutput >= 0.7) {
					pidRotateOutput = 0.7;
				}
				if (pidRotateOutput <= -0.7) {
					pidRotateOutput = -0.7;
				}
			}
		}
	}

	class VisionMove extends PIDSubsystem {
		public VisionMove(double p, double i, double d) {
			super("VisionMove", p, i, d);
			this.setSetpoint(visionMoveTargetWidth);
			getPIDController().setContinuous(false);
			LiveWindow.addActuator("VisionMove", "pid", getPIDController());
		}

		@Override
		protected double returnPIDInput() {
			if (!isAutonomous() && !isTest()) {
				return 0;
			}
			if (centerWidth > visionMoveTargetWidth) {
				visionMode = VisionMode.DONE;
			} else if (rotateAvgError > 60 && centerWidth < visionMoveTargetWidth * .75) {
				visionMode = VisionMode.ROTATE;
			}
			return centerWidth;
		}

		@Override
		protected void usePIDOutput(double output) {
			pidMoveOutput = 0;
			if (visionMode != VisionMode.MOVE) {
				return;
			}
			pidMoveOutput = output;
			if (pidMoveOutput >= 0.7) {
				pidMoveOutput = 0.7;
			}
			if (pidMoveOutput <= 0) {
				pidMoveOutput = 0;
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

	public static double rotateKp = 0.011;
	public static double rotateKi = 1.8E-4;
	public static double rotateKd = .007;
	public static double moveKp = .03;
	public static double moveKi = 0;
	public static double moveKd = .037;
	public static double visionMoveTargetWidth = 120;
	public static double maxRotateError = 15;
	public static double cameraWidth = 320;
	public static double cameraHeight = 240;
	public static double cameraCenterX = cameraWidth / 2.0;
	public static double cameraCenterY = cameraHeight / 2.0;

	public static OI oi;
	public static Robot instance;
	public static final DriveTrain drive = new DriveTrain();

	VisionRotate visionSteering;
	VisionMove visionDrive;
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
		visionSteering = new VisionRotate(rotateKp, rotateKi, rotateKd);
		visionDrive = new VisionMove(moveKp, moveKi, moveKd);
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
		LiveWindow.run();
		getCenters();
		++period;

		if (visionMode != VisionMode.DONE) {
			System.out.printf("%.2f Mode=%s Rotate=%.1f Move=%.1f targetX=%.1f Width=%.1f\n",
					System.currentTimeMillis() / 1000., visionMode, pidRotateOutput, pidMoveOutput, targetX,
					centerWidth);
			drive.arcadeDrive(pidMoveOutput, -pidRotateOutput);
		}
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

		// TODO: Check to see that the two largest areas are approx. equal
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
