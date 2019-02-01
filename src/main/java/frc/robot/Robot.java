/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Motor.EncoderError;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;

public class Robot extends edu.wpi.first.wpilibj.TimedRobot {

	private final RobotMap robot = new RobotMap();

	private final double targetTick = -4000;
	private MiniPID pid;

	private int stage = 0;

	private final ArrayList<Trajectory> leftPaths = new ArrayList<>(), rightPaths = new ArrayList<>();

	public Robot() {
		super(0.04d);
	}

	@Override
	public void robotInit() {
		try {
			this.robot.leftDrive.zeroEncoder();
			this.robot.rightDrive.zeroEncoder();
		} catch (EncoderError e) {
			e.printStackTrace();
		}

		this.pid = new MiniPID(0.0001d, 0.0000075d, 0);
		this.pid.setOutputLimits(-1.0d, 1.0d);
	}

	@Override
	public void robotPeriodic() {

		SmartDashboard.putNumber("Left position", this.robot.leftDrive.getPosition());
		SmartDashboard.putNumber("Right position", this.robot.rightDrive.getPosition());

		SmartDashboard.putNumber("Left drive power", this.robot.leftDrive.getMotorOutputPercent());
		SmartDashboard.putNumber("Right drive power", this.robot.rightDrive.getMotorOutputPercent());

	}

	@Override
	public void autonomousInit() {
		try {
			this.robot.leftDrive.zeroEncoder();
			this.robot.rightDrive.zeroEncoder();
		} catch (Exception e) {
			e.printStackTrace();
			return;
		}

		// Add the forward and reverse portions
		this.addPath("Forward", "turn");

		this.stage = 0;
	}

	private void addPath(String... names) {
		for (String name : names) {
			this.leftPaths.add(PathfinderFRC.getTrajectory(name + ".left"));
			this.rightPaths.add(PathfinderFRC.getTrajectory(name + ".right"));
		}
	}

	@Override
	public void autonomousPeriodic() {
		try {

			// Check if were still running off of stages
			if (stage < this.leftPaths.size() && this.stage < this.rightPaths.size()) {

				// Set it to run to those targeted positions
				this.robot.leftDrive.driveToPosition(this.getTargetPosition(this.leftPaths.get(stage)));
				this.robot.rightDrive.driveToPosition(this.getTargetPosition(this.rightPaths.get(stage)));

				// Check if the target has been reacked (within a certain descrepancy)
				if (this.robot.leftDrive.targetReached(50) && this.robot.rightDrive.targetReached(50)) {
					System.out.println("Done!");
					this.robot.leftDrive.zeroEncoder();
					this.robot.rightDrive.zeroEncoder();
					this.stage++;
				}

			} else {
				this.robot.leftDrive.stop();
				this.robot.rightDrive.stop();
			}

			SmartDashboard.putNumber("Right target", this.robot.rightDrive.getTarget());
			SmartDashboard.putNumber("Left target", this.robot.leftDrive.getTarget());

			SmartDashboard.putNumber("Right displacement",
					Math.abs(this.robot.rightDrive.getTarget() - this.robot.rightDrive.getPosition()));
			SmartDashboard.putNumber("Left displacement",
					Math.abs(this.robot.leftDrive.getTarget() - this.robot.leftDrive.getPosition()));
			SmartDashboard.putNumber("Stage", stage);
		} catch (EncoderError e) {
			e.printStackTrace();
		}
	}

	private double getTargetPosition(Trajectory trajectory) {
		// Calculate the left and right targets
		final int last = trajectory.length() - 1;
		double target = trajectory.get(last).position - trajectory.get(0).position;

		// Determine if its forward or backward
		double score = (trajectory.get(last).x - trajectory.get(0).x) + (trajectory.get(last).y - trajectory.get(0).y);
		if (score < 0) {
			target *= -1;
		}

		return target;
	}

	@Override
	public void teleopInit() {

	}

	@Override
	public void teleopPeriodic() {

		// Calculate drive code, with turbo button
		double forward = this.robot.gamepad1.getStickButton(Hand.kLeft) ? this.robot.gamepad1.getY(Hand.kLeft)
				: this.robot.gamepad1.getY(Hand.kLeft) / 2,
				turn = this.robot.gamepad1.getStickButton(Hand.kLeft) ? this.robot.gamepad1.getX(Hand.kRight)
						: this.robot.gamepad1.getX(Hand.kRight) / 2;

		// Basic west coast drive code
		if (Math.abs(forward) > 0.05d || Math.abs(turn) > 0.05d) {
			this.robot.leftDrive.setPower(forward - turn);
			this.robot.rightDrive.setPower(forward + turn);
		} else {
			this.robot.leftDrive.stop();
			this.robot.rightDrive.stop();
		}

	}

	@Override
	public void testInit() {
		try {
			this.robot.leftDrive.zeroEncoder();
			this.robot.rightDrive.zeroEncoder();
		} catch (Motor.EncoderError encoder) {
			System.err.println("This motor does not have an encoder");
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	@Override
	public void testPeriodic() {
		this.robot.leftDrive.setPower(this.pid.getOutput(this.robot.leftDrive.getPosition(), this.targetTick));
		this.robot.rightDrive.setPower(this.pid.getOutput(this.robot.rightDrive.getPosition(), this.targetTick));
	}

	@Override
	public void disabledInit() {
		this.robot.leftDrive.stop();
		this.robot.rightDrive.stop();
	}

	@Override
	public void disabledPeriodic() {
		// This function is me.
	}

}
