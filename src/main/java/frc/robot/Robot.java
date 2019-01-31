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

		// Add the forward portion
		this.leftPaths.add(PathfinderFRC.getTrajectory("Forward.left"));
		this.rightPaths.add(PathfinderFRC.getTrajectory("Forward.right"));

		// Add the reverse portion
		this.leftPaths.add(PathfinderFRC.getTrajectory("Reverse.left"));
		this.rightPaths.add(PathfinderFRC.getTrajectory("Reverse.right"));

		this.stage = 0;
	}

	@Override
	public void autonomousPeriodic() {
		try {

			// Check if were still running off of stages
			if (stage < this.leftPaths.size() && this.stage < this.rightPaths.size()) {

				// Calculate the left and right targets
				double leftTarget = this.leftPaths.get(stage).get(this.leftPaths.get(stage).length() - 1).position
						- this.leftPaths.get(stage).get(0).position;

				double rightTarget = this.rightPaths.get(stage).get(this.rightPaths.get(stage).length() - 1).position
						- this.rightPaths.get(stage).get(0).position;

				// Set it to run to those targeted positions
				this.robot.leftDrive.driveToPosition(leftTarget);
				this.robot.rightDrive.driveToPosition(rightTarget);

				// Check if the target has been reacked (within a certain descrepancy)
				if (this.robot.leftDrive.targetReached(50) && this.robot.rightDrive.targetReached(50)) {
					System.out.println("Done!");
					this.stage++;
				}

			} else {
				this.robot.leftDrive.stop();
				this.robot.rightDrive.stop();
			}

			SmartDashboard.putNumber("Right target", this.robot.rightDrive.getClosedLoopTarget());
			SmartDashboard.putNumber("Left target", this.robot.leftDrive.getClosedLoopTarget());

			SmartDashboard.putNumber("Right displacement",
					Math.abs(this.robot.rightDrive.getClosedLoopTarget() - this.robot.rightDrive.getPosition()));
			SmartDashboard.putNumber("Left displacement",
					Math.abs(this.robot.leftDrive.getClosedLoopTarget() - this.robot.leftDrive.getPosition()));
		} catch (EncoderError e) {
			e.printStackTrace();
		}
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
