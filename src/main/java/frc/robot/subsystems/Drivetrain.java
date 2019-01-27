package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Drivetrain {
    private TalonSRX left, right;

    public Drivetrain(TalonSRX left, TalonSRX right) {
        this.left = left;
        this.right = right;
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        left.set(ControlMode.PercentOutput, leftSpeed);
        right.set(ControlMode.PercentOutput, rightSpeed);
    }

    public void arcadeDrive(double forwardSpeed, double turnSpeed) {
        tankDrive(forwardSpeed + turnSpeed, forwardSpeed - turnSpeed);
    }

    public void stop() {
        left.set(ControlMode.PercentOutput, 0.0);
        right.set(ControlMode.PercentOutput, 0.0);
    }
}