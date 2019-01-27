package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.pidf.Gains;
import frc.robot.pidf.PIDF;
import frc.robot.utils.Bounds;
import frc.robot.utils.Utils;

public class Elevator {
    private TalonSRX winch;
    private DigitalInput limitSwitch;

    private double startTime;
    private PIDF pid;
    private double targetPosition;

    private Bounds positionBounds = new Bounds(0.0, 1.0);
    private Bounds sensorBounds = new Bounds(0.0, 20.0);

    public Elevator(TalonSRX winchMotor, DigitalInput limitSwitch) {
        this.winch = winchMotor;
        this.limitSwitch = limitSwitch;

        Bounds bounds = new Bounds(-0.4, 0.4);
        Gains gains = new Gains(0.005, 0.0, 0.0);
        pid = new PIDF(gains, bounds);

        targetPosition = positionBounds.min();
    }

    public void init() {
        startTime = Timer.getFPGATimestamp();
        pid.initialize(getPosition(), 0.0, 0.0);
    }

    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
    }

    public double getPosition() {
        double raw = (double) winch.getSelectedSensorPosition();
        return Utils.lerp(raw, sensorBounds.min(), sensorBounds.max(), 0.0, 1.0);
    }

    private void zeroSensor() {
        winch.setSelectedSensorPosition((int) sensorBounds.min());
    }

    public void update() {
        if (limitSwitch.get()) {
            zeroSensor();
        }

        double motorSpeed = pid.calculateOutput(getPosition(), targetPosition, Timer.getFPGATimestamp() - startTime);
        winch.set(ControlMode.PercentOutput, motorSpeed);
    }
}