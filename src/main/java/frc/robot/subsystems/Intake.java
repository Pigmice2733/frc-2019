package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.motion.Setpoint;
import frc.robot.motion.StaticProfile;
import frc.robot.motion.execution.StaticProfileExecutor;
import frc.robot.pidf.Gains;
import frc.robot.pidf.PIDF;
import frc.robot.utils.Bounds;
import frc.robot.utils.NTStreamer;
import frc.robot.utils.Utils;

public class Intake {
    public class Target {
        public static final double START = 0.0;
        public static final double INTAKE = 0.545;
        public static final double STOWED_UP = 0.365;
        public static final double STOWED_BACK = 0.06;
    }

    private TalonSRX pivot;
    private TalonSRX roller;

    private Double targetPosition;

    private Bounds sensorBounds = new Bounds(0, 2640.0);

    private StaticProfileExecutor profileExecutor;
    private NTStreamer<Double> positionStreamer;
    private NTStreamer<Double> targetStreamer;
    private NTStreamer<Double> setpointStreamer;
    private NTStreamer<Double> outputStreamer;
    private AHRS navx;

    private double kF = 0.75;

    private PIDF balancer;

    public Intake(TalonSRX pivotMotor, TalonSRX rollerMotor, AHRS navx) {
        pivot = pivotMotor;
        roller = rollerMotor;
        this.navx = navx;
        pivot.config_kP(0, 1.2, 10);
        pivot.config_kI(0, 0.001, 10);
        pivot.config_kD(0, 1.2, 10);
        pivot.config_kF(0, 0.0, 10);

        positionStreamer = new NTStreamer<>("intake", "position");
        targetStreamer = new NTStreamer<>("intake", "target");
        setpointStreamer = new NTStreamer<>("intake", "setpoint");
        outputStreamer = new NTStreamer<>("intake", "output");

        zeroSensor();
        setTargetPosition(Target.STOWED_BACK);

        Gains gains = new Gains(0.1, 0.0, 0.0);
        Bounds outputBounds = new Bounds(-0.4, 0.4);
        balancer = new PIDF(gains, outputBounds);
    }

    public void drive(double percent) {
        outputStreamer.send(percent);
        pivot.set(ControlMode.PercentOutput, percent);
        positionStreamer.send(getPosition());
    }

    public void setRoller(double percent) {
        roller.set(ControlMode.PercentOutput, percent);
    }

    public void startBalancing() {
        balancer.initialize(navx.getRoll(), Timer.getFPGATimestamp(), 0.0);
    }

    public void levelRobot() {
        pivot.set(ControlMode.PercentOutput, -balancer.calculateOutput(navx.getRoll(), -3, Timer.getFPGATimestamp()));
    }

    public void setTargetPosition(double targetPosition) {
        if (profileExecutor == null || Math.abs(this.targetPosition - targetPosition) > 1e-2) {
            resetPID();
            this.targetPosition = targetPosition;
            StaticProfile profile = new StaticProfile(getVelocity(), getPosition(), targetPosition, 1.4, 2.0, 0.8);
            profileExecutor = new StaticProfileExecutor(profile, this::output, Timer::getFPGATimestamp, 0.02);
            profileExecutor.initialize();
        }
    }

    public double getPosition() {
        double raw = (double) pivot.getSelectedSensorPosition();
        return Utils.lerp(raw, sensorBounds.min(), sensorBounds.max(), 0.0, 1.0);
    }

    public double getVelocity() {
        double raw = (double) pivot.getSelectedSensorVelocity();
        return Utils.lerp(raw, -sensorBounds.size(), sensorBounds.size(), -1.0, 1.0);
    }

    public void zeroSensor() {
        pivot.setSelectedSensorPosition((int) sensorBounds.min());
    }

    public void updateSensor() {
        positionStreamer.send(getPosition());
        targetStreamer.send(this.targetPosition);
    }

    public void update() {
        updateSensor();

        profileExecutor.update();
    }

    public void resetPID() {
        pivot.setIntegralAccumulator(0.0, 0, 10);
    }

    private void output(Setpoint sp) {
        double lerp = Utils.lerp(sp.getPosition(), 0.0, 1.0, sensorBounds.min(), sensorBounds.max());
        double angle = Utils.lerp(getPosition(), 0.29, 0.62, 0.5 * Math.PI, Math.PI);
        double gravityCompensation = 0.075 * Math.cos(angle);
        setpointStreamer.send(sp.getPosition());
        pivot.set(ControlMode.Position, lerp, DemandType.ArbitraryFeedForward,
                gravityCompensation + (kF * sp.getVelocity()));
        outputStreamer.send(pivot.getMotorOutputPercent());
    }
}