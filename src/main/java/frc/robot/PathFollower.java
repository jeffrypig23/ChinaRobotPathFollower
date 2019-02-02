package frc.robot;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

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

	private final int k_ticks_per_rev = 1240;
	private final double k_wheel_diameter = 4.0d;
	private final double k_max_velocity = 204.0d;

	public EncoderFollower leftFollower, rightFollower;

	private RobotMap robot;

	public PathFollower(RobotMap robot) {
		this.robot = robot;
	}

	public void loadPath(String pathName, boolean forward) {

		Trajectory left_trajectory = PathfinderFRC.getTrajectory(pathName + ".right"),
				right_trajectory = PathfinderFRC.getTrajectory(pathName + ".left");
		// If we are going forward, invert the positions in the trajectory, if not, just
		// flip the sides
		if (forward) {
			for (Segment segment : left_trajectory.segments) {
				segment.position *= -1;
			}
			for (Segment segment : right_trajectory.segments) {
				segment.position *= -1;
			}
		}

		this.leftFollower = new EncoderFollower(left_trajectory);
		this.rightFollower = new EncoderFollower(right_trajectory);

		this.leftFollower.configureEncoder(0, this.k_ticks_per_rev, this.k_wheel_diameter);
		// You must tune the PID values on the following line!
		this.leftFollower.configurePIDVA(0.0175, 0.0, 0.0, 1 / this.k_max_velocity, 0);

		this.rightFollower.configureEncoder(0, this.k_ticks_per_rev, this.k_wheel_diameter);
		// You must tune the PID values on the following line!
		this.rightFollower.configurePIDVA(0.0175, 0.0, 0.0, 1 / this.k_max_velocity, 0);

		this.robot.leftDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 40);
		this.robot.rightDrive.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 40);
	}
}