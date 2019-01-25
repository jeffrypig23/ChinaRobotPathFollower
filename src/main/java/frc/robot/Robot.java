/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Motor.EncoderError;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

public class Robot extends TimedRobot {

	private final RobotMap robot = new RobotMap();

	private final double targetTick = 4000;
	private MiniPID pid;

	private double left_target, right_target;

	private final double k_ticks_per_rev = 1413;
	private final double k_wheel_diameter = 0.1016d; // 4 inches to meters
	private final double k_max_velocity = 0.4572; // 18 inches to meters
	private final String k_path_name = "forward";

	private AHRS m_gyro;

	private EncoderFollower m_left_follower;
	private EncoderFollower m_right_follower;

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

		this.m_gyro = new AHRS(SPI.Port.kMXP);
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

		System.out.printf("Left target (inches): %s\nRIght target (inches): %s\n", this.left_target, this.right_target);

		// this.robot.leftDrive.driveToPosition(this.left_target);
		/*
		 * this.m_left_follower = new EncoderFollower(left_trajectory);
		 * this.m_right_follower = new EncoderFollower(right_trajectory);
		 * 
		 * // Be sure that the talons are updating correctly
		 * this.robot.leftDrive1.setStatusFramePeriod(StatusFrame.Status_2_Feedback0,
		 * 40);
		 * this.robot.rightDrive1.setStatusFramePeriod(StatusFrame.Status_2_Feedback0,
		 * 40);
		 * 
		 * m_left_follower.configureEncoder(this.robot.leftDrive1.
		 * getSelectedSensorPosition(), (int) this.k_ticks_per_rev,
		 * this.k_wheel_diameter); // You must tune the PID values on the following
		 * line! m_left_follower.configurePIDVA(0.0001d, 0.0000075d, 0.0, 1 /
		 * this.k_max_velocity, 0);
		 * 
		 * m_right_follower.configureEncoder(this.robot.rightDrive1.
		 * getSelectedSensorPosition(), (int) this.k_ticks_per_rev,
		 * this.k_wheel_diameter); // You must tune the PID values on the following //
		 * line! m_right_follower.configurePIDVA(0.0001d, 0.0000075d, 0.0, 1 /
		 * this.k_max_velocity, 0);
		 * 
		 * this.m_follower_notifier = new Notifier(this::followPath);
		 * this.m_follower_notifier.startPeriodic(PathfinderFRC.getTrajectory(
		 * k_path_name).get(0).dt);
		 */

	}

	@Override
	public void testPeriodic() {
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

	@Deprecated
	private void followPath() {
		if (this.m_left_follower.isFinished() || this.m_right_follower.isFinished()) {
			this.m_follower_notifier.stop();

			// Stop the motors
			this.robot.leftDrive.stop();
			this.robot.rightDrive.stop();
		} else {

			try {
				// Calculate the speeds for the left and right side
				double left_speed = this.m_left_follower.calculate((int) this.robot.leftDrive.getPosition());
				double right_speed = this.m_right_follower.calculate((int) this.robot.rightDrive.getPosition());

				// Also dislpay the position of the segment for each side
				SmartDashboard.putNumber("Left position", this.m_left_follower.getSegment().position);
				SmartDashboard.putNumber("Right position", this.m_right_follower.getSegment().position);

				// Calculate a slight turn if needed
				double heading = m_gyro.getAngle();
				double desired_heading = Pathfinder.r2d(m_left_follower.getHeading());
				double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
				double turn = 0.8 * (-1.0 / 80.0) * heading_difference;

				// Apply the resulting power
				this.robot.leftDrive.setPower(-(left_speed + turn));
				this.robot.rightDrive.setPower(-(right_speed - turn));
			} catch (ArrayIndexOutOfBoundsException e) {
				this.m_follower_notifier.stop();

				// Stop the motors
				this.robot.leftDrive.stop();
				this.robot.rightDrive.stop();
			}

		}
	}

}
