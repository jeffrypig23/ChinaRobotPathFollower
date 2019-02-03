/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.LinkedList;
import java.util.Queue;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Motor.EncoderError;

public class Robot extends edu.wpi.first.wpilibj.TimedRobot {

	private final RobotMap robot = new RobotMap();

	// Variables for tracking velocity
	private long initialTime;
	private boolean trackVelocity = false;
	private double initialVelocityPosition = 0.0;

	// Variables for following a set path
	private PathFollower follower = new PathFollower(this.robot);
	private Queue<String[]> paths = new LinkedList<>();

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
	}

	@Override
	public void robotPeriodic() {
		SmartDashboard.putNumber("Left position", this.robot.leftDrive.getPosition());
		SmartDashboard.putNumber("Right position", this.robot.rightDrive.getPosition());
		SmartDashboard.putNumber("Left drive power", this.robot.leftDrive.getMotorOutputPercent());
		SmartDashboard.putNumber("Right drive power", this.robot.rightDrive.getMotorOutputPercent());
		SmartDashboard.putNumber("Heading", this.robot.navX.getFusedHeading());
	}

	@Override
	public void autonomousInit() {
		try {
			// Reset the encoders
			this.robot.leftDrive.zeroEncoder();
			this.robot.rightDrive.zeroEncoder();

			// Load the paths into a queue
			this.paths.add(new String[] { "one", "true" });
			this.paths.add(new String[] { "two", "false" });
			this.paths.add(new String[] { "one", "true" });
			this.paths.add(new String[] { "two", "false" });

			// Setup load the first path to drive to
			String[] path = this.paths.poll();
			this.follower.loadPath(path[0], Boolean.parseBoolean(path[1]));
		} catch (Exception e) {
			e.printStackTrace();
			return;
		}
	}

	@Override
	public void autonomousPeriodic() {
		try {

			if (this.follower != null) {
				if (this.follower.leftFollower != null && this.follower.rightFollower != null) {
					// Check if the path has been completed
					if (this.follower.leftFollower.isFinished() && this.follower.rightFollower.isFinished()) {
						// Stop motors and load the next path
						this.robot.leftDrive.stop();
						this.robot.rightDrive.stop();
						if (this.paths.size() > 0) {
							this.robot.leftDrive.zeroEncoder();
							this.robot.rightDrive.zeroEncoder();
							String[] path = this.paths.poll();
							this.follower.loadPath(path[0], Boolean.parseBoolean(path[1]));
						}
					} else {
						// Drive based on the path
						double left_speed = this.follower.leftFollower
								.calculate(this.robot.leftDrive.getSelectedSensorPosition());
						double right_speed = this.follower.rightFollower
								.calculate(this.robot.rightDrive.getSelectedSensorPosition());
						this.robot.leftDrive.setPower(left_speed);
						this.robot.rightDrive.setPower(right_speed);
					}
				}
			} else {
				// Something is wrong, so stop.
				this.robot.leftDrive.stop();
				this.robot.rightDrive.stop();
			}

			// Update telemetry
			//this.updateAutonomousTelemetry();

		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	private void updateAutonomousTelemetry() {
		SmartDashboard.putNumber("Right target", this.follower.rightFollower.getSegment().position);
		SmartDashboard.putNumber("Left target", this.follower.leftFollower.getSegment().position);
		SmartDashboard.putNumber("Right displacement",
				Math.abs(this.follower.rightFollower.getSegment().position - this.robot.rightDrive.getPosition()));
		SmartDashboard.putNumber("Left displacement",
				Math.abs(this.follower.rightFollower.getSegment().position - this.robot.leftDrive.getPosition()));
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

		// Check if we are tracking velocity
		if (this.trackVelocity) {
			// Check if one second has elapsed
			if (System.currentTimeMillis() >= initialTime + 1000) {
				System.out.println(
						"Current velocity: " + (this.robot.leftDrive.getPosition() - this.initialVelocityPosition));
				this.trackVelocity = false;
			}
		} else {
			// Check if we want to start tracking velocity
			if (this.robot.gamepad1.getBumperPressed(Hand.kLeft) || this.robot.gamepad1.getBumperPressed(Hand.kRight)) {

				// Set the initial position
				this.initialVelocityPosition = this.robot.leftDrive.getPosition();

				// Set the check time
				this.initialTime = System.currentTimeMillis();

				System.out.println("Tracking velocity...");
				this.trackVelocity = true;
			}
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
		this.robot.leftDrive.stop();
		this.robot.rightDrive.stop();
		this.paths.clear();
	}

	@Override
	public void disabledPeriodic() {
		// This function is me.
	}

}
