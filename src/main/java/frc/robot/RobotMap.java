package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.XboxController;

public class RobotMap {

    // First, start with the ports for the Chicago robot
    private final int leftDrive1Port = 3, leftDrive2Port = 2, rightDrive1Port = 10, rightDrive2Port = 9,
            leftIntakePort = 8, rightIntakePort = 7, liftDrive1Port = 5, liftDrive2Port = 6, liftDrive3Port = 4,
            wristPort = 11;

    // Now, delcare the drive motors that are on the robot
    public TalonSRX leftDrive1, leftDrive2, rightDrive1, rightDrive2;

    // Also setup the controllers for the drivers
    public XboxController gamepad1 = new XboxController(0), gamepad2 = new XboxController(1);

    public RobotMap() {

        // Set up drive motors
        this.leftDrive1 = new TalonSRX(this.leftDrive1Port);
        this.leftDrive2 = new TalonSRX(this.leftDrive2Port);
        this.rightDrive1 = new TalonSRX(this.rightDrive1Port);
        this.rightDrive2 = new TalonSRX(this.rightDrive2Port);

        // Setup encoders
        this.leftDrive1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        this.rightDrive1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

        // Set secondary motors to follower masters
        this.leftDrive2.set(ControlMode.Follower, this.leftDrive1Port);
        this.rightDrive2.set(ControlMode.Follower, this.rightDrive1Port);

        // Invert necessary drive motors
        this.leftDrive1.setInverted(true);
        this.leftDrive2.setInverted(true);
        
        this.leftDrive1.setSensorPhase(true);
		this.rightDrive1.setSensorPhase(true);
    }

}