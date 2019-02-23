package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Outtake {
    private TalonSRX motor;

    public Outtake(TalonSRX motor) {
        this.motor = motor;
    }

    public void drive(double speed) {
        motor.set(ControlMode.PercentOutput, speed);
    }
}