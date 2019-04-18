package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.motion.Setpoint;
import frc.robot.pidf.Gains;
import frc.robot.pidf.PIDF;
import frc.robot.utils.Bounds;
import frc.robot.utils.Odometry;

public class Drivetrain {
    private TalonSRX leftDrive, rightDrive;
    private AHRS navx;
    private double trackwidth;

    PIDF visionAlignment;

    private Odometry nonLinearStateEstimator;

    private boolean visionEngaged = false;

    // Convert between feet and encoder ticks
    private static final double ticksPerFoot = 4096 / (Math.PI * 0.5);

    public Drivetrain(TalonSRX leftDrive, TalonSRX rightDrive, AHRS navx, double trackwidth) {
        this.leftDrive = leftDrive;
        this.rightDrive = rightDrive;
        this.navx = navx;
        this.trackwidth = trackwidth;

        Bounds visionOutputBounds = new Bounds(-0.6, 0.6);
        Gains alignmentGains = new Gains(-0.012, 0.0, 0.0);
        visionAlignment = new PIDF(alignmentGains, visionOutputBounds);

        // this.nonLinearStateEstimator = new Odometry(0.0, 0.0, navx.getAngle());
    }

    public double getTrackWidth() {
        return trackwidth;
    }

    public void initializePID() {
        leftDrive.setSelectedSensorPosition(0, 0, 100);
        rightDrive.setSelectedSensorPosition(0, 0, 100);
    }

    public void stop() {
        tankDrive(0.0, 0.0);
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        leftDrive.set(ControlMode.PercentOutput, leftSpeed);
        rightDrive.set(ControlMode.PercentOutput, rightSpeed);

        // nonLinearStateEstimator.update(getLeftSensorPosition(),
        // getRightSensorPosition(), navx.getAngle());
    }

    public void arcadeDrive(double forwardSpeed, double turnSpeed) {
        tankDrive(forwardSpeed + turnSpeed, forwardSpeed - turnSpeed);
    }

    public void visionDrive(double forwardSpeed, double driverSteer, boolean targetVisible, double targetAngle) {
        if (targetVisible) {
            if (!visionEngaged) {
                visionEngaged = true;
                visionAlignment.initialize(targetAngle, Timer.getFPGATimestamp(), 0.0);
            }

            double output = visionAlignment.calculateOutput(targetAngle, 0.0, Timer.getFPGATimestamp());
            arcadeDrive(forwardSpeed, output);
        } else {
            System.out.println("Target not visible");
            visionEngaged = false;
            arcadeDrive(forwardSpeed, driverSteer);
        }
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
