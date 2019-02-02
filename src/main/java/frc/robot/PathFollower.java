package frc.robot;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.wpilibj.Notifier;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;
import jaci.pathfinder.followers.EncoderFollower;

public class PathFollower {

	// Max velocity in ticks is 23000 ticks per second
	// Which is 21800 under load
	// Which is ~215 inches per second

	// Acceleration is 78.72 inche per second per second

	// Jerk is 2364 inches per second per second per second

	// FIXME
	private final int k_ticks_per_rev = 1240;
	private final double k_wheel_diameter = 4.0d;
	private final double k_max_velocity = 204.0d;

	private EncoderFollower m_left_follower;
	private EncoderFollower m_right_follower;

	public Notifier m_follower_notifier;

	private String name;
	private RobotMap robot;

	public PathFollower(String name, RobotMap robot) {
		this.name = name;
		this.robot = robot;
	}

	public void initPathFollower(boolean forward) {

		// Fix due to error with Jaci's code
		Trajectory left_trajectory = PathfinderFRC.getTrajectory(this.name + ".right");
		Trajectory right_trajectory = PathfinderFRC.getTrajectory(this.name + ".left");

		// If we are going forward, invert the positions in the trajectory
		if (forward) {
			for (Segment segment : left_trajectory.segments) {
				segment.position *= -1;
			}
			for (Segment segment : right_trajectory.segments) {
				segment.position *= -1;
			}
		}

		this.m_left_follower = new EncoderFollower(left_trajectory);
		this.m_right_follower = new EncoderFollower(right_trajectory);

		this.m_left_follower.configureEncoder(0, this.k_ticks_per_rev, this.k_wheel_diameter);
		// You must tune the PID values on the following line!
		this.m_left_follower.configurePIDVA(0.0175, 0.0, 0.0, 1 / this.k_max_velocity, 0);

		this.m_right_follower.configureEncoder(0, this.k_ticks_per_rev, this.k_wheel_diameter);
		// You must tune the PID values on the following line!
		this.m_right_follower.configurePIDVA(0.0175, 0.0, 0.0, 1 / this.k_max_velocity, 0);

		this.robot.leftDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 40);
		this.robot.rightDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 40);

		this.m_follower_notifier = new Notifier(this::followPath);
		this.m_follower_notifier.startPeriodic(left_trajectory.get(0).dt);
	}

	private void followPath() {
		if (this.m_left_follower.isFinished() || this.m_right_follower.isFinished()) {
			this.m_follower_notifier.stop();
			this.robot.leftDrive.stop();
			this.robot.rightDrive.stop();
		} else {

			double leftInches = ((double) (this.robot.leftDrive.getSelectedSensorPosition()) / this.k_ticks_per_rev)
					* Math.PI * this.k_wheel_diameter;
			System.out.println(String.format("%s L-> %s", leftInches, this.m_left_follower.getSegment().position));

			double rightInches = ((double) (this.robot.rightDrive.getSelectedSensorPosition()) / this.k_ticks_per_rev)
					* Math.PI * this.k_wheel_diameter;
			System.out.println(String.format("%s R-> %s", rightInches, this.m_right_follower.getSegment().position));

			System.out.println("");

			// Remember, this needs to be inverted
			double left_speed = this.m_left_follower.calculate(this.robot.leftDrive.getSelectedSensorPosition());
			double right_speed = this.m_right_follower.calculate(this.robot.rightDrive.getSelectedSensorPosition());
			double heading = this.robot.navX.getFusedHeading() - 180;
			double desired_heading = Pathfinder.r2d(this.m_left_follower.getHeading());
			double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
			double turn = 0.8d * (1.0d / 80.0d) * heading_difference;
			this.robot.leftDrive.setPower(left_speed);
			this.robot.rightDrive.setPower(right_speed);
		}
	}

}