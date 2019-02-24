package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class Outtake {
    private VictorSPX motor;

    public Outtake(VictorSPX motor) {
        this.motor = motor;
    }

    public void drive(double speed) {
        motor.set(ControlMode.PercentOutput, speed);
    }
}