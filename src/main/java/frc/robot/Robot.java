/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Motor.EncoderError;
import frc.robot.autonomousOptions.autonomous;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;

public class Robot extends edu.wpi.first.wpilibj.TimedRobot {

	private final RobotMap robot = new RobotMap();

	private final double targetTick = -4000;
	private MiniPID pid;

	private int stage = 0;

	private long initialTime;

	private boolean trackVelocity = false;

	private double initialVelocityPosition = 0.0;

	private final ArrayList<Trajectory> leftPaths = new ArrayList<>(), rightPaths = new ArrayList<>();

	private final SendableChooser<autonomous> autoChooser = new SendableChooser<>();

	private PathFollower follower;

	public Robot() {
		super(0.04d);
	}

	@Override
	public void robotInit() {
		try {
			this.robot.leftDrive.zeroEncoder();
			this.robot.rightDrive.zeroEncoder();

			this.autoChooser.setDefaultOption("PID", autonomous.PID);
			this.autoChooser.addOption("MP off path", autonomous.MP);
			this.autoChooser.addOption("Pathfollower", autonomous.PATH);

			SmartDashboard.putData(this.autoChooser);

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

		SmartDashboard.putNumber("Heading", this.robot.navX.getFusedHeading());
	}

	@Override
	public void autonomousInit() {
		try {
			this.robot.leftDrive.zeroEncoder();
			this.robot.rightDrive.zeroEncoder();

			// Get the option which was chosen from the sendable chooser
			switch (this.autoChooser.getSelected()) {
			case MP:
				// Add the forward and reverse portions
				this.addPath("Forward", "turn", "Forward");

				this.stage = 0;
				break;
			case PID:
				// Do nothing
				break;
			case PATH:
				this.follower = new PathFollower(this.robot);
				this.stage = 0;
				break;
			case OTHER:
				// Do nothing
				break;

			}
		} catch (Exception e) {
			e.printStackTrace();
			return;
		}
	}

	private void addPath(String... names) {
		for (String name : names) {
			// Becasue the drive wheels are inverted, we also have to invert the sides
			this.leftPaths.add(PathfinderFRC.getTrajectory(name + ".right"));
			this.rightPaths.add(PathfinderFRC.getTrajectory(name + ".left"));
		}
	}

	@Override
	public void autonomousPeriodic() {
		try {

			switch (this.autoChooser.getSelected()) {
			case MP:
				this.motionProfileAuto();
				break;
			case PID:
				this.pidAuto();
				break;
			case PATH:
				switch (stage) {
				case 0:
					// Go the the frst path (one), then wait for completion
					this.robot.leftDrive.zeroEncoder();
					this.robot.rightDrive.zeroEncoder();
					this.follower.initPathFollower("one", true);
					stage++;
					break;
				case 1:
					// Check if the stage has completed, then start the next one
					if (this.follower.pathFinished) {
						this.robot.leftDrive.zeroEncoder();
						this.robot.rightDrive.zeroEncoder();
						this.follower.initPathFollower("two", false);
						stage++;
					}
					break;
				case 2:
					// Check if the stage has completed, then start the next one
					if (this.follower.pathFinished) {
						this.robot.leftDrive.zeroEncoder();
						this.robot.rightDrive.zeroEncoder();
						this.follower.initPathFollower("one", true);
						stage++;
					}
					break;
				case 3:
					// Check if the stage has completed, then start the next one
					if (this.follower.pathFinished) {
						this.robot.leftDrive.zeroEncoder();
						this.robot.rightDrive.zeroEncoder();
						this.follower.initPathFollower("two", false);
						stage++;
					}
					break;
				case 4:
					if (this.follower.pathFinished) {
						System.out.println("Go to next path");
						stage++;
					}
					break;
				}
				break;
			case OTHER:
				break;
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

	private void motionProfileAuto() throws EncoderError {
		if (stage < this.leftPaths.size() && this.stage < this.rightPaths.size()) {

			// Set it to run to those targeted positions
			this.robot.leftDrive.driveToPosition(this.getTargetPosition(this.leftPaths.get(stage)));
			this.robot.rightDrive.driveToPosition(this.getTargetPosition(this.rightPaths.get(stage)));

			// Check if the target has been reacked (within a certain descrepancy)
			if (this.robot.leftDrive.targetReached(107) && this.robot.rightDrive.targetReached(107)) {
				System.out.println("Done!");
				this.robot.leftDrive.zeroEncoder();
				this.robot.rightDrive.zeroEncoder();
				this.stage++;
			}

		} else {
			this.robot.leftDrive.stop();
			this.robot.rightDrive.stop();
		}
	}

	private void pidAuto() {
		this.robot.leftDrive.setPower(this.pid.getOutput(this.robot.leftDrive.getPosition(), this.targetTick));
		this.robot.rightDrive.setPower(this.pid.getOutput(this.robot.rightDrive.getPosition(), this.targetTick));
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
		this.rightPaths.clear();
		this.leftPaths.clear();
		if (this.follower != null) {
			if (this.follower.m_follower_notifier != null) {
				this.follower.m_follower_notifier.stop();
			}
		}
	}

	@Override
	public void disabledPeriodic() {
		// This function is me.
	}

}
