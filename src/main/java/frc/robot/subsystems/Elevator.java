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

    private double targetPosition;

    // physical max: 30100
    // physical min: 0
    private Bounds sensorBounds = new Bounds(0, 30300.0);

    private StaticProfileExecutor profileExecutor;
    private NTStreamer<Double> positionStreamer;
    private NTStreamer<Double> targetStreamer;
    private NTStreamer<Double> setpointStreamer;
    private NTStreamer<Double> outputStreamer;

    private double kF = 0.8;

    public Elevator(TalonSRX winchMotor) {
        this.winch = winchMotor;
        winchMotor.config_kP(0, 0.0009);
        winchMotor.config_kI(0, 0.000001);
        winchMotor.config_kD(0, 0.0);
        winchMotor.config_kF(0, 0.0);

        positionStreamer = new NTStreamer<>("elevator", "position");
        targetStreamer = new NTStreamer<>("elevator", "target");
        setpointStreamer = new NTStreamer<>("elevator", "setpoint");
        outputStreamer = new NTStreamer<>("elevator", "output");

        this.targetPosition = 1.0;
        zeroSensor();
        setTargetPosition(0.0);
    }

    public void init() {
        setTargetPosition(getPosition());
    }

    public void setTargetPosition(double targetPosition) {
        if (Math.abs(this.targetPosition - targetPosition) > 1e-2) {
            this.targetPosition = targetPosition;
            System.out.println("Profiling from " + getPosition() + " to " + targetPosition);
            StaticProfile profile = new StaticProfile(getVelocity(), getPosition(), targetPosition, 0.4, 0.2, 0.2);
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
        winch.setSelectedSensorPosition((int) sensorBounds.min());
    }

    public void updateSensor() {
        // if (limitSwitch.get()) {
        // zeroSensor();
        // }

        positionStreamer.send(getPosition());
        targetStreamer.send(this.targetPosition);
    }

    public void update() {
        updateSensor();

        profileExecutor.update();
    }

    private void output(Setpoint sp) {
        double lerp = Utils.lerp(sp.getPosition(), 0.0, 1.0, sensorBounds.min(), sensorBounds.max());
        setpointStreamer.send(sp.getPosition());
        winch.set(ControlMode.Position, lerp, DemandType.ArbitraryFeedForward, kF * sp.getVelocity() + 0.03);
        outputStreamer.send(winch.getMotorOutputPercent());
    }
}