package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.motion.Setpoint;
import frc.robot.motion.execution.StaticProfileExecutor;
import frc.robot.motion.StaticProfile;
import frc.robot.pidf.Gains;
import frc.robot.pidf.PIDF;
import frc.robot.utils.Bounds;
import frc.robot.utils.NTStreamer;
import frc.robot.utils.Utils;

public class Arm {
    public class Target {
        public static final double ANGLED_DOWN = 0.05;
        public static final double ANGLED_UP = 0.85;
        public static final double UP_FLAT = 0.95;
        public static final double DOWN_FLAT = 0.15;
    }

    private TalonSRX pivot;

    private Double targetPosition;

    private Bounds sensorBounds = new Bounds(0, 9500.0);

    private StaticProfileExecutor profileExecutor;
    private NTStreamer<Double> positionStreamer;
    private NTStreamer<Double> targetStreamer;
    private NTStreamer<Double> setpointStreamer;
    private NTStreamer<Double> outputStreamer;

    private double kF = 0.4;

    public Arm(TalonSRX pivotMotor) {
        this.pivot = pivotMotor;
        pivot.config_kP(0, 0.0);
        pivot.config_kI(0, 0.0);
        pivot.config_kD(0, 0.0);
        pivot.config_kF(0, 0.0);

        positionStreamer = new NTStreamer<>("arm", "position");
        targetStreamer = new NTStreamer<>("arm", "target");
        setpointStreamer = new NTStreamer<>("arm", "setpoint");
        outputStreamer = new NTStreamer<>("arm", "output");

        targetPosition = Target.DOWN_FLAT;

        zeroSensor();
        setTargetPosition(0.0);
    }

    public void drive(double percent) {
        outputStreamer.send(percent);
        pivot.set(ControlMode.PercentOutput, percent);
    }

    public void setTargetPosition(double targetPosition) {
        if (this.targetPosition == null || Math.abs(this.targetPosition - targetPosition) > 1e-2) {
            this.targetPosition = targetPosition;
            System.out.println("Profiling arm from " + getPosition() + " to " + targetPosition);
            StaticProfile profile = new StaticProfile(getVelocity(), getPosition(), targetPosition, 0.4, 0.2, 0.2);
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

    private void output(Setpoint sp) {
        double lerp = Utils.lerp(sp.getPosition(), 0.0, 1.0, sensorBounds.min(), sensorBounds.max());
        setpointStreamer.send(sp.getPosition());
        pivot.set(ControlMode.Position, lerp, DemandType.ArbitraryFeedForward, kF * sp.getVelocity() + 0.06);
        outputStreamer.send(pivot.getMotorOutputPercent());
    }
}