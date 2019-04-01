package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;

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
        public static final double INTAKE = 0.56;
        public static final double STOWED_FRONT = 0.375;
        public static final double STOWED_UP = 0.245;
        public static final double STOWED_BACK = 0.04;
        public static final double CLIMB = 0.39;
    }

    private CANSparkMax pivot;
    private TalonSRX sensor;
    private TalonSRX roller;

    private boolean levelling = false;

    private Double targetPosition;
    private Double currentPosition;

    private Bounds sensorBounds = new Bounds(0, 2640.0);
    private Bounds climbRange = new Bounds(0.39, 2.0);

    private AHRS navx;

    private StaticProfileExecutor profileExecutor;
    private double gravitykF = 0.075;
    private PIDF positionPID;
    // private NTStreamer<Double> positionStreamer;
    // private NTStreamer<Double> targetStreamer;
    // private NTStreamer<Double> setpointStreamer;
    // private NTStreamer<Double> outputStreamer;

    private PIDF balancePID;

    public Intake(CANSparkMax pivot, TalonSRX positionSensor, TalonSRX roller, AHRS navx) {
        this.pivot = pivot;
        this.sensor = positionSensor;
        this.roller = roller;
        this.navx = navx;

        // positionStreamer = new NTStreamer<>("intake", "position");
        // targetStreamer = new NTStreamer<>("intake", "target");
        // setpointStreamer = new NTStreamer<>("intake", "setpoint");
        // outputStreamer = new NTStreamer<>("intake", "output");

        zeroSensor();
        setTargetPosition(Target.STOWED_BACK);

        Gains gains = new Gains(0.15, 0.0, 0.0);
        Bounds outputBounds = new Bounds(-0.8, 0.8);
        balancePID = new PIDF(gains, outputBounds);

        gains = new Gains(0.1, 1e-4, 1.0, 0.0, 0.75, 0.0);
        outputBounds = new Bounds(-0.5, 0.5);
        positionPID = new PIDF(gains, outputBounds);
    }

    public void drive(double percent) {
        pivot.set(percent);
        levelling = false;
    }

    public void setRoller(double percent) {
        roller.set(ControlMode.PercentOutput, percent);
    }

    public void startBalancing() {
        balancePID.initialize(navx.getRoll(), Timer.getFPGATimestamp(), 0.0);
    }

    public void levelRobot() {
        if (!levelling) {
            startBalancing();
            levelling = true;
        }

        double output = -balancePID.calculateOutput(navx.getRoll(), -14, Timer.getFPGATimestamp());

        pivot.set(output);
    }

    public void setTargetPosition(double targetPosition) {
        levelling = false;
        if (profileExecutor == null || Math.abs(this.targetPosition - targetPosition) > 1e-2) {
            resetPID();
            this.targetPosition = targetPosition;
            StaticProfile profile = new StaticProfile(getVelocity(), getPosition(), targetPosition, 2.4, 2.4, 1.4);
            profileExecutor = new StaticProfileExecutor(profile, this::output, this::getPosition, 0.02);
            profileExecutor.initialize();
        }
    }

    public double getPosition() {
        double raw = (double) sensor.getSelectedSensorPosition();
        currentPosition = Utils.lerp(raw, sensorBounds.min(), sensorBounds.max(), 0.0, 1.0);
        return currentPosition;
    }

    public double getVelocity() {
        double raw = (double) sensor.getSelectedSensorVelocity();
        return Utils.lerp(raw, -sensorBounds.size(), sensorBounds.size(), -1.0, 1.0);
    }

    public void zeroSensor() {
        sensor.setSelectedSensorPosition((int) sensorBounds.min());
        currentPosition = 0.0;
    }

    public void updateSensor() {
        currentPosition = getPosition();
    }

    public void update() {
        updateSensor();
        levelling = false;
        profileExecutor.updateNoEnd();
    }

    public void resetPID() {
        sensor.setIntegralAccumulator(0.0, 0, 10);
    }

    private void output(Setpoint sp) {
        double lerp = Utils.lerp(sp.getPosition(), 0.0, 1.0, sensorBounds.min(), sensorBounds.max());
        double angle = Utils.lerp(currentPosition, 0.29, 0.62, 0.5 * Math.PI, Math.PI);
        double gravityCompensation = gravitykF * Math.cos(angle);
        double output = positionPID.calculateOutput(currentPosition, lerp, sp.getVelocity(), sp.getAcceleration(),
                Timer.getFPGATimestamp()) + gravityCompensation;
        pivot.set(output);
    }
}