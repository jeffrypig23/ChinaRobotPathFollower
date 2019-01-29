package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

public class Motor extends com.ctre.phoenix.motorcontrol.can.TalonSRX {

    /**
     * How many ticks there are in one inch
     */
    private final double ticks_per_inch = 353.25d * Math.PI;

    /**
     * Boolean used to make sure that the setup in <code>driveToPosition</code> only
     * runs once. After that, it sets this boolean to false.
     */
    private boolean driveFirstRun = true;

    /**
     * Target encoder position (in ticks) used in <code>driveToPosition</code>.
     */
    private double target;

    /**
     * Constructor for the motor object (custom TalonSRX object).
     * 
     * @param port The motor port (ID on the CAN bus)
     */
    public Motor(int port) {
        super(port);
    }

    // FIXME
    public void driveToPosition(double position) throws EncoderError {
        this.target = -(this.ticks_per_inch / position);

        // Only run this once to set up the talon stuff.
        // While zeroEncoder() resets the driveFristRun variable to true, we reset it at
        // the very end to false, causing this to only run once.
        // One way to reset this is to re-declare the motor, or to zero the encoder
        if (this.driveFirstRun) {
            // https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java/MotionMagic/src/main/java/frc/robot/Robot.java
            // https://phoenix-documentation.readthedocs.io/en/latest/ch16_ClosedLoop.html#sensor-preparation

            this.configFactoryDefault();

            this.selectProfileSlot(0, 0);
            this.configNominalOutputForward(0);
            this.configNominalOutputReverse(0);
            this.configPeakOutputForward(1);
            this.configPeakOutputReverse(-1);

            this.config_kP(0, 0.0001);
            this.config_kI(0, 0.0000075d);
            this.config_kD(0, 0d);
            this.config_kF(0, 0.0d);

            this.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 0);
            this.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 0);

            this.configMotionCruiseVelocity(20000);
            this.configMotionAcceleration(6000);

            this.zeroEncoder();

            this.driveFirstRun = false;
            System.out.println("Finished setup");
        }

        this.set(ControlMode.MotionMagic, this.target);
    }

    /**
     * Checks if the current motor's position has reached its set target position.
     * 
     * @return True if the motors current position equals the target position.
     *         Otherwise it returns false.
     */
    public boolean targetReached() {
        return this.target == this.getPosition();
    }

    public boolean targetReached(int error) {
        return Math.abs(this.target - this.getPosition()) <= error;
    }

    /**
     * Sets the motor output to 0, essentially acting as a break.
     */
    public void stop() {
        this.set(ControlMode.PercentOutput, 0.0d);
    }

    /**
     * Sets the power of the motor to whatever percentage (0 to 1) was provided. A
     * negative number will result in the motor driving in reverse.
     * 
     * @param power The power percentage to supply to the motor (0 to 1)
     */
    public void setPower(double power) {
        if (power > 1.0d) {
            power = 1.0d;
        } else if (power < -1.0d) {
            power = 1.0d;
        }

        this.set(ControlMode.PercentOutput, power);
    }

    /**
     * Returns the motor's current encoder position (in ticks).
     * 
     * @return The current encoder position.
     */
    public double getPosition() {
        return ((double) this.getSelectedSensorPosition());
    }

    public void zeroEncoder() throws EncoderError {
        if (!this.setSelectedSensorPosition(0, 0, 0).equals(com.ctre.phoenix.ErrorCode.OK)) {
            throw new EncoderError("Cannot zero encoder. Does this motor not have one?");
        } else {
            this.driveFirstRun = true;
            return;
        }
    }

    public class EncoderError extends Exception {
        static final long serialVersionUID = 10252;

        public EncoderError(String message) {
            super(message);
        }
    }
}