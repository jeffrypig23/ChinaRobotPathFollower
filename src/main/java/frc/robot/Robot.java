/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Motor.EncoderError;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;

public class Robot extends edu.wpi.first.wpilibj.TimedRobot {

	private final RobotMap robot = new RobotMap();

	private final double targetTick = 4000;
	private MiniPID pid;

	private double left_target, right_target;

	// private final double k_ticks_per_rev = 1413;
	// private final double k_wheel_diameter = 0.1016d; // 4 inches to meters
	// private final double k_max_velocity = 0.4572; // 18 inches to meters
	private final String k_path_name = "forward";

	private Notifier m_follower_notifier;

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
		} catch (Motor.EncoderError encoder) {
			System.err.println("This motor does not have an encoder");
		} catch (Exception e) {
			e.printStackTrace();
		}

	}

	@Override
	public void autonomousPeriodic() {
		this.robot.leftDrive.setPower(-this.pid.getOutput(this.robot.leftDrive.getPosition(), this.targetTick));
		this.robot.rightDrive.setPower(-this.pid.getOutput(this.robot.rightDrive.getPosition(), this.targetTick));
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
		} catch (Exception e) {
			e.printStackTrace();
			return;
		}

		Trajectory left_trajectory = PathfinderFRC.getTrajectory(k_path_name + ".left");
		Trajectory right_trajectory = PathfinderFRC.getTrajectory(k_path_name + ".right");

		// Get the change in position
		this.left_target = left_trajectory.get(left_trajectory.length() - 1).position - left_trajectory.get(0).position;

		this.right_target = right_trajectory.get(right_trajectory.length() - 1).position
				- right_trajectory.get(0).position;

		System.out.printf("Left target (inches): %s\nRight target (inches): %s\n", this.left_target, this.right_target);
	}

	@Override
	public void testPeriodic() {
		try {
			this.robot.leftDrive.driveToPosition(this.left_target);
			this.robot.rightDrive.driveToPosition(this.right_target);
		} catch (EncoderError e) {
			e.printStackTrace();
		}
	}

	@Override
	public void disabledInit() {
		if (this.m_follower_notifier != null) {
			this.m_follower_notifier.stop();
		}
		this.robot.leftDrive.stop();
		this.robot.rightDrive.stop();
	}

	@Override
	public void disabledPeriodic() {
		// This function is me.
	}

}
