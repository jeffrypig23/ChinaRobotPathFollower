package frc.robot;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Motor extends TalonSRX {

    private final double ticks_per_inch = 353.25d * Math.PI;

    private boolean driveFirstRun = true;

    private double target;

    public Motor(int port) {
        super(port);
    }

    public void driveToPosition(double position) throws EncoderError {
        this.target = this.ticks_per_inch / position;

        // Only run this once to set up the talon stuff.
        // While zeroEncoder() resets the driveFristRun variable to true, we reset it at
        // the very end to false, causing this to only run once.
        // One way to reset this is to re-declare the motor, or to zero the encoder
        if (this.driveFirstRun) {
            // https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java/MotionMagic/src/main/java/frc/robot/Robot.java
            this.configNominalOutputForward(0);
            this.configNominalOutputReverse(0);
            this.configPeakOutputForward(1);
            this.configPeakOutputReverse(-1);

            this.config_kP(0, 0.0001d);
            this.config_kI(0, 0.0000075d);
            this.config_kD(0, 0);
            this.config_kF(0, 0);

            this.configMotionCruiseVelocity((int) Math.round(18 * this.ticks_per_inch));
            this.configMotionAcceleration(6000);

            this.zeroEncoder();
            this.driveFirstRun = false;
        }

        this.set(ControlMode.MotionMagic, this.target);
    }

    public boolean targetReached() {
        return this.target == this.getPosition();
    }

    public boolean targetReached(int error) {
        return Math.abs(this.target - this.getPosition()) <= error;
    }

    public void stop() {
        this.set(ControlMode.PercentOutput, 0.0d);
    }

    public void setPower(double power) {
        if (power > 1.0d) {
            power = 1.0d;
        } else if (power < -1.0d) {
            power = 1.0d;
        }

        this.set(ControlMode.PercentOutput, power);
    }

    public double getPosition() {
        return ((double) this.getSelectedSensorPosition());
    }

    public void zeroEncoder() throws EncoderError {
        if (!this.setSelectedSensorPosition(0).equals(ErrorCode.OK)) {
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