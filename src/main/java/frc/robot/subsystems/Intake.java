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

public class Intake {
    public class Target {
        public static final double INTAKE = 0.53;
        public static final double STOWED_UP = 0.34;
        public static final double STOWED_BACK = 0.0;
    }

    private TalonSRX pivot;
    private TalonSRX roller;

    private Double targetPosition = null;

    private Bounds sensorBounds = new Bounds(0, 2640.0);

    private StaticProfileExecutor profileExecutor;
    private NTStreamer<Double> positionStreamer;
    private NTStreamer<Double> targetStreamer;
    private NTStreamer<Double> setpointStreamer;
    private NTStreamer<Double> outputStreamer;

    private double kF = 0.2;

    public Intake(TalonSRX pivotMotor, TalonSRX rollerMotor) {
        this.pivot = pivotMotor;
        this.roller = rollerMotor;
        pivot.config_kP(0, 0.2, 10);
        pivot.config_kI(0, 0.0, 10);
        pivot.config_kD(0, 0.0, 10);
        pivot.config_kF(0, 0.0, 10);

        positionStreamer = new NTStreamer<>("intake", "position");
        targetStreamer = new NTStreamer<>("intake", "target");
        setpointStreamer = new NTStreamer<>("intake", "setpoint");
        outputStreamer = new NTStreamer<>("intake", "output");

        targetPosition = Target.STOWED_BACK;

        zeroSensor();
        setTargetPosition(0.0);
    }

    public void drive(double percent) {
        outputStreamer.send(percent);
        pivot.set(ControlMode.PercentOutput, percent);
        positionStreamer.send(getPosition());
    }

    public void setRoller(double percent) {
        roller.set(ControlMode.PercentOutput, percent);
    }

    public void setTargetPosition(double targetPosition) {
        System.out.println("target position" + (targetPosition + this.targetPosition));
        if (this.targetPosition == null || Math.abs(this.targetPosition - targetPosition) > 1e-2) {
            this.targetPosition = targetPosition;
            System.out.println("Profiling intake from " + getPosition() + " to " + targetPosition);
            StaticProfile profile = new StaticProfile(getVelocity(), getPosition(), targetPosition, 0.65, 1.4, 1.4);
            profileExecutor = new StaticProfileExecutor(profile, this::output, Timer::getFPGATimestamp, 0.02);
            profileExecutor.initialize();
            System.out.println("Hey");
        } else {
            System.out.println("Wat " + this.targetPosition);
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
        setpointStreamer.send(sp.getPosition());
        pivot.set(ControlMode.Position, lerp, DemandType.ArbitraryFeedForward, kF * sp.getVelocity());
        outputStreamer.send(pivot.getMotorOutputPercent());
    }
}