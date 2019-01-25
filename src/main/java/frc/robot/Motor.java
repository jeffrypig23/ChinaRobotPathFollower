package frc.robot;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Motor extends TalonSRX {

    private final double ticks_per_inch = 0.0d;

    private int target;

    public Motor(int port) {
        super(port);
    }

    public void driveToPosition(int position) {
        this.target = position;
        // TODO
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
        }
    }

    public class EncoderError extends Exception {
        static final long serialVersionUID = 10252;

        public EncoderError(String message) {
            super(message);
        }
    }

}