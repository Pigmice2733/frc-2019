package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.motion.Setpoint;
import frc.robot.motion.StaticProfile;
import frc.robot.motion.execution.StaticProfileExecutor;
import frc.robot.utils.Bounds;
import frc.robot.utils.Utils;

public class Elevator {
    private TalonSRX winch;
    private DigitalInput limitSwitch;

    private double targetPosition;

    private Bounds positionBounds = new Bounds(0.0, 1.0);
    private Bounds sensorBounds = new Bounds(0.0, 20.0);

    private StaticProfileExecutor profileExecutor;

    public Elevator(TalonSRX winchMotor, DigitalInput limitSwitch) {
        this.winch = winchMotor;
        this.limitSwitch = limitSwitch;

        targetPosition = positionBounds.min();
    }

    public void init() {
        setTargetPosition(getPosition());
    }

    public void setTargetPosition(double targetPosition) {
        if (this.targetPosition != targetPosition) {
            this.targetPosition = targetPosition;
            StaticProfile profile = new StaticProfile(getVelocity(), getPosition(), targetPosition, 0.5, 0.2, 0.2);
            profileExecutor = new StaticProfileExecutor(profile, this::output, Timer::getFPGATimestamp, 0.02);
        }
    }

    public double getPosition() {
        double raw = (double) winch.getSelectedSensorPosition();
        return Utils.lerp(raw, sensorBounds.min(), sensorBounds.max(), 0.0, 1.0);
    }

    public double getVelocity() {
        double raw = (double) winch.getSelectedSensorVelocity();
        return Utils.lerp(raw, sensorBounds.min(), sensorBounds.max(), -1.0, 1.0);
    }

    private void zeroSensor() {
        winch.setSelectedSensorPosition((int) sensorBounds.min());
    }

    public void update() {
        if (limitSwitch.get()) {
            zeroSensor();
        }

        profileExecutor.update();
    }

    private void output(Setpoint sp) {
        winch.set(ControlMode.Position, sp.getPosition(), DemandType.ArbitraryFeedForward, sp.getVelocity());
    }
}