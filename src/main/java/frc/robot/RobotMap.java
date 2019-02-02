package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;

public class RobotMap {

    // First, start with the ports for the Chicago robot
    private final int leftDrive1Port = 3, leftDrive2Port = 2, rightDrive1Port = 10,
            rightDrive2Port = 9/* , liftDrive1Port = 5, liftDrive2Port = 6, liftDrive3Port = 4 */;

    // Now, delcare the drive motors that are on the robot
    public Motor leftDrive, rightDrive;

    private Motor leftDrive2, rightDrive2;

    // Also setup the controllers for the drivers
    public XboxController gamepad1 = new XboxController(0), gamepad2 = new XboxController(1);

    public AHRS navX;

    public RobotMap() {

        // Set up drive motors
        this.leftDrive = new Motor(this.leftDrive1Port);
        this.leftDrive2 = new Motor(this.leftDrive2Port);
        this.rightDrive = new Motor(this.rightDrive1Port);
        this.rightDrive2 = new Motor(this.rightDrive2Port);

        // Setup encoders
        this.leftDrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        this.rightDrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

        // Set secondary motors to follower masters
        this.leftDrive2.set(ControlMode.Follower, this.leftDrive1Port);
        this.rightDrive2.set(ControlMode.Follower, this.rightDrive1Port);

        // Invert necessary drive motors
        this.leftDrive.setInverted(true);
        this.leftDrive2.setInverted(true);

        // Invert the encoders
        this.leftDrive.setSensorPhase(true);
        this.rightDrive.setSensorPhase(true);

        // Setup the navX
        this.navX = new AHRS(SPI.Port.kMXP);

    }

}