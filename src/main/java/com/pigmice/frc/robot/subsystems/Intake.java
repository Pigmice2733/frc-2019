package com.pigmice.frc.robot.subsystems;

import java.text.DecimalFormat;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.pigmice.frc.lib.motion.Setpoint;
import com.pigmice.frc.lib.motion.StaticProfile;
import com.pigmice.frc.lib.motion.execution.StaticProfileExecutor;
import com.pigmice.frc.lib.pidf.Gains;
import com.pigmice.frc.lib.pidf.PIDF;
import com.pigmice.frc.lib.utils.Range;
import com.pigmice.frc.lib.utils.Utils;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Timer;

public class Intake {
    public class Target {
        public static final double START = 0.0;
        public static final double INTAKE = 0.575;
        public static final double STOWED_FRONT = 0.39;
        public static final double STOWED_UP = 0.245;
        public static final double STOWED_BACK = 0.05;
        public static final double CLIMB = 0.39;
    }

    private CANSparkMax pivot, follower;
    private CANEncoder sensor;
    private TalonSRX roller;

    private boolean levelling = false;
    private boolean rateExceeded = false;

    private Double targetPosition;
    private Double currentPosition;

    private Range sensorBounds = new Range(0, 53.0);
    private Range climbRange = new Range(0.425, 1.4);

    private double maximumRate = 0.5;
    private double positionDelta;

    private AHRS navx;

    private StaticProfileExecutor profileExecutor;
    private double gravitykF = 0.02;
    private PIDF positionPID;
    // private NTStreamer<Double> positionStreamer;
    // private NTStreamer<Double> targetStreamer;
    // private NTStreamer<Double> setpointStreamer;
    // private NTStreamer<Double> outputStreamer;

    private PIDF balancePID;

    DecimalFormat df = new DecimalFormat("#.##");

    private final boolean intakeIsInstalled;
    private double mockIntakePosition = 0.0;

    public static Intake UninstalledIntake() {
        return new Intake();
    }

    private Intake() {
        intakeIsInstalled = false;
    }

    public Intake(CANSparkMax pivot, CANSparkMax follower, CANEncoder positionSensor, TalonSRX roller, AHRS navx) {
        intakeIsInstalled = true;

        this.pivot = pivot;
        this.follower = follower;
        this.sensor = positionSensor;
        this.roller = roller;
        this.navx = navx;

        // positionStreamer = new NTStreamer<>("intake", "position");
        // targetStreamer = new NTStreamer<>("intake", "target");
        // setpointStreamer = new NTStreamer<>("intake", "setpoint");
        // outputStreamer = new NTStreamer<>("intake", "output");

        Gains gains = new Gains(0.2, 0.0, 0.0);
        Range outputBounds = new Range(-0.8, 0.8);
        balancePID = new PIDF(gains, outputBounds);

        gains = new Gains(2.4, 0, 0.0, 0.0, 0.5, 0.0);
        outputBounds = new Range(-0.5, 0.5);
        positionPID = new PIDF(gains, outputBounds);

        zeroSensor();
        currentPosition = Target.STOWED_BACK;
        positionDelta = 0.0;
        setTargetPosition(Target.STOWED_BACK);
    }

    public void drive(double percent) {
        if (intakeIsInstalled) {
            pivot.set(percent);
            follower.set(-percent);
            levelling = false;
            rateExceeded = false;
        }
    }

    public void setRoller(double percent) {
        if (intakeIsInstalled) {
            roller.set(ControlMode.PercentOutput, percent);
        }
    }

    public void startBalancing() {
        if (intakeIsInstalled) {
            balancePID.initialize(navx.getRoll(), Timer.getFPGATimestamp(), 0.0);
        }
    }

    public void levelRobot() {
        if (intakeIsInstalled) {
            if (!levelling) {
                startBalancing();
                levelling = true;
            }

            double output = -balancePID.calculateOutput(navx.getRoll(), -8, Timer.getFPGATimestamp());

            if (positionDelta > maximumRate || (currentPosition < climbRange.min() && output < 0.0)
                    || (currentPosition > climbRange.max() && output > 0.0)) {
                rateExceeded = true;
                pivot.set(0.0);
                follower.set(0.0);
            } else {
                if (rateExceeded) {
                    rateExceeded = false;
                    startBalancing();
                }

                if (getVelocity() > 0.5) {
                    output = Math.min(output, 0.25);
                }

                pivot.set(output);
                follower.set(-output);
            }
        }
    }

    public void setTargetPosition(double targetPosition) {
        if (intakeIsInstalled) {
            if (profileExecutor == null || Math.abs(this.targetPosition - targetPosition) > 1e-2 || levelling) {
                levelling = false;
                this.targetPosition = targetPosition;
                StaticProfile profile = new StaticProfile(getVelocity(), currentPosition, targetPosition, 1.8, 1.8,
                        0.8);
                profileExecutor = new StaticProfileExecutor(profile, this::output, this::getPosition, 0.02);
                positionPID.initialize(currentPosition, Timer.getFPGATimestamp(), 0.0);
                profileExecutor.initialize();
            }
        } else {
            mockIntakePosition = targetPosition;
        }
    }

    public double getPosition() {
        if (intakeIsInstalled) {
            double raw = (double) sensor.getPosition();
            currentPosition = Utils.lerp(raw, sensorBounds.min(), sensorBounds.max(), 0.0, 1.0);
            return currentPosition;
        } else {
            return mockIntakePosition;
        }
    }

    public double getVelocity() {
        if (intakeIsInstalled) {
            double raw = (double) sensor.getVelocity();
            return Utils.lerp(raw, -sensorBounds.size(), sensorBounds.size(), -1.0, 1.0) / 60.0;
        } else {
            return 0.0;
        }
    }

    public void zeroSensor() {
        if (intakeIsInstalled) {
            sensor.setPosition((int) sensorBounds.min());
            currentPosition = 0.0;
        }
    }

    public void updateSensor() {
        if (intakeIsInstalled) {
            double previousPosition = currentPosition;
            currentPosition = getPosition();
            positionDelta = Math.abs(currentPosition - previousPosition);
        }
    }

    public void update() {
        if (intakeIsInstalled) {
            levelling = false;

            if (positionDelta < maximumRate || currentPosition < 0.28) {
                if (rateExceeded) {
                    rateExceeded = false;
                    profileExecutor = null;
                    setTargetPosition(targetPosition);
                }
                profileExecutor.updateNoEnd();
            } else {
                rateExceeded = true;
            }
        }
    }

    private void output(Setpoint sp) {
        if (intakeIsInstalled) {
            double angle = Utils.lerp(currentPosition, 0.265, 0.635, 0.5 * Math.PI, Math.PI);
            double gravityCompensation = gravitykF * Math.cos(angle);
            double output = positionPID.calculateOutput(currentPosition, sp.getPosition(), sp.getVelocity(),
                    sp.getAcceleration(), Timer.getFPGATimestamp()) + gravityCompensation;

            if (currentPosition > 0.025 || output > 0.0) {
                pivot.set(output);
                follower.set(-output);
            }
        }
    }
}
