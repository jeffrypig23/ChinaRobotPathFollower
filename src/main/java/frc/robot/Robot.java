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
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

public class Robot extends TimedRobot {

	private final RobotMap robot = new RobotMap();

	private final double targetTick = 3000;
	private MiniPID pid;

	private final double k_ticks_per_rev = 1413;
	private final double k_wheel_diameter = 0.1016d; // 4 inches to meters
	private final double k_max_velocity = 0.9144d; // 36 inches to meters
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
		this.robot.leftDrive1.setSelectedSensorPosition(0);
		this.robot.rightDrive1.setSelectedSensorPosition(0);
		this.m_gyro = new AHRS(SPI.Port.kMXP);
		this.pid = new MiniPID(0.0001d, 0, 0);
		this.pid.setOutputLimits(-1.0d, 1.0d);
	}

	@Override
	public void robotPeriodic() {

		SmartDashboard.putNumber("Left position", this.robot.leftDrive1.getSelectedSensorPosition());
		SmartDashboard.putNumber("Right position", this.robot.rightDrive1.getSelectedSensorPosition());

		SmartDashboard.putNumber("Left drive power", this.robot.leftDrive1.getMotorOutputPercent());
		SmartDashboard.putNumber("Right drive power", this.robot.rightDrive1.getMotorOutputPercent());

	}

	@Override
	public void autonomousInit() {
		try {
			this.robot.leftDrive1.setSelectedSensorPosition(0);
			this.robot.rightDrive1.setSelectedSensorPosition(0);
			/*
			 * Trajectory left_trajectory = PathfinderFRC.getTrajectory(k_path_name +
			 * ".left"); Trajectory right_trajectory =
			 * PathfinderFRC.getTrajectory(k_path_name + ".right");
			 * 
			 * this.m_left_follower = new EncoderFollower(left_trajectory);
			 * this.m_right_follower = new EncoderFollower(right_trajectory);
			 * 
			 * // Be sure that the talons are updating correctly
			 * this.robot.leftDrive1.setStatusFramePeriod(StatusFrame.Status_2_Feedback0,
			 * 20);
			 * this.robot.rightDrive1.setStatusFramePeriod(StatusFrame.Status_2_Feedback0,
			 * 20);
			 * 
			 * m_left_follower.configureEncoder(this.robot.leftDrive1.
			 * getSelectedSensorPosition(), (int) this.k_ticks_per_rev,
			 * this.k_wheel_diameter); // You must tune the PID values on the following
			 * line! m_left_follower.configurePIDVA(1.0, 0.0, 0.0, 1 / this.k_max_velocity,
			 * 0);
			 * 
			 * m_right_follower.configureEncoder(this.robot.rightDrive1.
			 * getSelectedSensorPosition(), (int) this.k_ticks_per_rev,
			 * this.k_wheel_diameter); // You must tune the PID values on the following
			 * line! m_right_follower.configurePIDVA(1.0, 0.0, 0.0, 1 / this.k_max_velocity,
			 * 0);
			 * 
			 * this.m_follower_notifier = new Notifier(this::followPath);
			 * this.m_follower_notifier.startPeriodic(PathfinderFRC.getTrajectory(
			 * k_path_name).get(0).dt);
			 */

		} catch (Exception e) {
			e.printStackTrace();
		}

	}

	@Override
	public void autonomousPeriodic() {
		this.robot.leftDrive1.set(ControlMode.PercentOutput,
				-this.pid.getOutput(this.robot.leftDrive1.getSelectedSensorPosition(), this.targetTick));
		this.robot.rightDrive1.set(ControlMode.PercentOutput,
				-this.pid.getOutput(this.robot.rightDrive1.getSelectedSensorPosition(), this.targetTick));
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
			this.robot.leftDrive1.set(ControlMode.PercentOutput, forward - turn);
			this.robot.rightDrive1.set(ControlMode.PercentOutput, forward + turn);
		} else {
			this.robot.leftDrive1.set(ControlMode.PercentOutput, 0);
			this.robot.rightDrive1.set(ControlMode.PercentOutput, 0);
		}

	}

	@Override
	public void testInit() {

	}

	@Override
	public void testPeriodic() {

	}

	@Override
	public void disabledInit() {
		if (this.m_follower_notifier != null) {
			this.m_follower_notifier.stop();
			this.robot.leftDrive1.set(ControlMode.PercentOutput, 0);
			this.robot.rightDrive1.set(ControlMode.PercentOutput, 0);
		}
	}

	@Override
	public void disabledPeriodic() {
		// This function is me.
	}

	private void followPath() {
		if (this.m_left_follower.isFinished() || this.m_right_follower.isFinished()) {
			this.m_follower_notifier.stop();

			// Stop the motors
			this.robot.leftDrive1.set(ControlMode.PercentOutput, 0);
			this.robot.rightDrive1.set(ControlMode.PercentOutput, 0);
		} else {

			// Calculate the speeds for the left and right side
			double left_speed = this.m_left_follower.calculate(this.robot.leftDrive1.getSelectedSensorPosition());
			double right_speed = this.m_right_follower.calculate(this.robot.rightDrive1.getSelectedSensorPosition());

			// Also dislpay the position of the segment for each side
			SmartDashboard.putNumber("Left position", this.m_left_follower.getSegment().position);
			SmartDashboard.putNumber("Right position", this.m_right_follower.getSegment().position);

			// Calculate a slight turn if needed
			double heading = m_gyro.getAngle();
			double desired_heading = Pathfinder.r2d(m_left_follower.getHeading());
			double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
			double turn = 0.8 * (-1.0 / 80.0) * heading_difference;

			// Apply the resulting power
			this.robot.leftDrive1.set(ControlMode.PercentOutput, left_speed - turn);
			this.robot.rightDrive1.set(ControlMode.PercentOutput, right_speed + turn);

		}
	}

}
