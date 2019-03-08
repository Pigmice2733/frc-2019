package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.motion.Setpoint;
import frc.robot.motion.StaticProfile;
import frc.robot.motion.execution.StaticProfileExecutor;
import frc.robot.utils.Bounds;
import frc.robot.utils.NTStreamer;
import frc.robot.utils.Utils;

public class Elevator {
    private TalonSRX winch;

    private Double targetPosition;

    private Double currentPosition;

    private Bounds sensorBounds = new Bounds(0, 30400.0);

    private StaticProfileExecutor profileExecutor;
    private NTStreamer<Double> positionStreamer;
    private NTStreamer<Double> targetStreamer;
    private NTStreamer<Double> setpointStreamer;
    private NTStreamer<Double> outputStreamer;

    private double kF = 0.75;
    private double gravityCompensation = 0.055;

    public Elevator(TalonSRX winchMotor) {
        winch = winchMotor;
        winchMotor.config_kP(0, 0.2, 10);
        winchMotor.config_kI(0, 0.0, 10);
        winchMotor.config_kD(0, 0.0, 10);
        winchMotor.config_kF(0, 0.0, 10);
        winchMotor.configNeutralDeadband(0.005, 10);

        positionStreamer = new NTStreamer<>("elevator", "position");
        targetStreamer = new NTStreamer<>("elevator", "target");
        setpointStreamer = new NTStreamer<>("elevator", "setpoint");
        outputStreamer = new NTStreamer<>("elevator", "output");

        zeroSensor();
        setTargetPosition(0.138);
    }

    public void drive(double percent) {
        outputStreamer.send(percent);
        winch.set(ControlMode.PercentOutput, percent);
    }

    public void setTargetPosition(double targetPosition) {
        if (profileExecutor == null || Math.abs(this.targetPosition - targetPosition) > 1e-2) {
            resetPID();
            StaticProfile profile;
            if (this.targetPosition != null && this.targetPosition > targetPosition) {
                profile = new StaticProfile(getVelocity(), getPosition(), targetPosition, 1.3, 0.9, 0.9);
            } else {
                profile = new StaticProfile(getVelocity(), getPosition(), targetPosition, 1.3, 0.9, 0.9);
            }
            this.targetPosition = targetPosition;
            profileExecutor = new StaticProfileExecutor(profile, this::output, Timer::getFPGATimestamp, 0.02);
            profileExecutor.initialize();
        }
    }

    public double getPosition() {
        double raw = (double) winch.getSelectedSensorPosition();
        return Utils.lerp(raw, sensorBounds.min(), sensorBounds.max(), 0.0, 1.0);
    }

    public double getVelocity() {
        double raw = (double) winch.getSelectedSensorVelocity();
        return Utils.lerp(raw, -sensorBounds.size(), sensorBounds.size(), -1.0, 1.0);
    }

    public void zeroSensor() {
        currentPosition = 0.138;
        winch.setSelectedSensorPosition((int) Utils.lerp(0.138, 0, 1.0, sensorBounds.min(), sensorBounds.max()));
    }

    public void updateSensor() {
        currentPosition = getPosition();
        positionStreamer.send(currentPosition);
        targetStreamer.send(this.targetPosition);
    }

    public void update() {
        updateSensor();

        profileExecutor.update();
    }

    public void resetPID() {
        winch.setIntegralAccumulator(0.0, 0, 10);
    }

    private void output(Setpoint sp) {
        double lerp = Utils.lerp(sp.getPosition(), 0.0, 1.0, sensorBounds.min(), sensorBounds.max());
        setpointStreamer.send(sp.getPosition());
        winch.set(ControlMode.Position, lerp, DemandType.ArbitraryFeedForward,
                gravityCompensation + kF * sp.getVelocity());
        outputStreamer.send(winch.getMotorOutputPercent());
    }
}