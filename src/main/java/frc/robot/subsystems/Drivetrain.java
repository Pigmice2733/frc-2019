package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.motion.Setpoint;
import frc.robot.utils.Odometry;

public class Drivetrain {
    private TalonSRX leftDrive, rightDrive;
    private AHRS navx;
    private double trackwidth;

    private Odometry nonLinearStateEstimator;

    // Convert between feet and encoder ticks
    private static final double ticksPerFoot = 4096 / (Math.PI * 0.5);

    public Drivetrain(TalonSRX leftDrive, TalonSRX rightDrive, AHRS navx, double trackwidth) {
        this.leftDrive = leftDrive;
        this.rightDrive = rightDrive;
        this.navx = navx;
        this.trackwidth = trackwidth;

        // this.nonLinearStateEstimator = new Odometry(0.0, 0.0, navx.getAngle());
    }

    public double getTrackWidth() {
        return trackwidth;
    }

    public void initializePID() {
        leftDrive.setSelectedSensorPosition(0, 0, 100);
        rightDrive.setSelectedSensorPosition(0, 0, 100);
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        leftDrive.set(ControlMode.PercentOutput, leftSpeed);
        rightDrive.set(ControlMode.PercentOutput, rightSpeed);

        // nonLinearStateEstimator.update(getLeftSensorPosition(), getRightSensorPosition(), navx.getAngle());
    }

    public void arcadeDrive(double forwardSpeed, double turnSpeed) {
        tankDrive(forwardSpeed + turnSpeed, forwardSpeed - turnSpeed);
    }

    public void stop() {
        tankDrive(0.0, 0.0);
    }

    public void PIDDrive(Setpoint leftSetpoint, Setpoint rightSetpoint) {
        double leftPosition = leftSetpoint.getPosition();
        double feedforward = leftSetpoint.getVelocity();
        leftDrive.set(ControlMode.Position, leftPosition * ticksPerFoot, DemandType.ArbitraryFeedForward,
                feedforward * ticksPerFoot);

        double rightPosition = rightSetpoint.getPosition();
        feedforward = rightSetpoint.getVelocity();
        rightDrive.set(ControlMode.Position, rightPosition * ticksPerFoot, DemandType.ArbitraryFeedForward,
                feedforward * ticksPerFoot);

        nonLinearStateEstimator.update(leftPosition, rightPosition, navx.getAngle());
    }

    public double getSensorPosition() {
        double avg = 0.5 * (leftDrive.getSelectedSensorPosition(0) + rightDrive.getSelectedSensorPosition(0));
        return avg / ticksPerFoot;
    }

    public double getLeftSensorPosition() {
        return leftDrive.getSelectedSensorPosition(0) / ticksPerFoot;
    }

    public double getRightSensorPosition() {
        return rightDrive.getSelectedSensorPosition(0) / ticksPerFoot;
    }

    public double getSensorVelocity() {
        double avg = 0.5 * (leftDrive.getSelectedSensorVelocity(0) + rightDrive.getSelectedSensorVelocity(0));
        return avg / ticksPerFoot;
    }
}
