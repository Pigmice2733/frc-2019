package com.pigmice.frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.pigmice.frc.lib.motion.Setpoint;
import com.pigmice.frc.lib.motion.StaticProfile;
import com.pigmice.frc.lib.motion.execution.StaticProfileExecutor;
import com.pigmice.frc.lib.utils.Range;
import com.pigmice.frc.lib.utils.Utils;

public class Arm {
    public class Target {
        public static final double START = -0.0175;
        public static final double CLIMB = 0.0;
        public static final double DOWN_ANGLE = 0.02;
        public static final double UP_ANGLE = 0.955;
        public static final double UP_FLAT = 0.905;
        public static final double DOWN_FLAT = 0.0775; // Pro
        public static final double DOWN_UP = 0.14;
        public static final double DOWN_SLIGHT = 0.048;
        public static final double CARGO_OUTTAKE = 0.24;
        public static final double CARGO_SHIP = 0.15;
    }

    private TalonSRX pivot;

    private Double targetPosition;

    private Double currentPosition;

    private static double verticalPosition = 0.779;

    private Range sensorBounds = new Range(0, 9500.0);

    private StaticProfileExecutor profileExecutor;
    // private NTStreamer<Double> positionStreamer;
    // private NTStreamer<Double> targetStreamer;
    // private NTStreamer<Double> setpointStreamer;
    // private NTStreamer<Double> outputStreamer;
    // private NTStreamer<Double> angleStreamer;

    private double kF = 0.4;

    public Arm(TalonSRX pivotMotor) {
        pivot = pivotMotor;
        pivot.config_kP(0, 0.7, 10);
        pivot.config_kI(0, 0.0, 10);
        pivot.config_kD(0, 0.0, 10);
        pivot.config_kF(0, 0.0, 10);

        // positionStreamer = new NTStreamer<>("arm", "position");
        // targetStreamer = new NTStreamer<>("arm", "target");
        // setpointStreamer = new NTStreamer<>("arm", "setpoint");
        // outputStreamer = new NTStreamer<>("arm", "output");
        // angleStreamer = new NTStreamer<>("arm", "angle");

        targetPosition = Target.DOWN_FLAT;

        zeroSensor();
        setTargetPosition(0.0);
    }

    public void drive(double percent) {
        pivot.set(ControlMode.PercentOutput, percent);
    }

    public void setTargetPosition(double targetPosition) {
        if (profileExecutor == null || Math.abs(this.targetPosition - targetPosition) > 1e-2) {
            resetPID();
            this.targetPosition = targetPosition;
            StaticProfile profile = new StaticProfile(getVelocity(), currentPosition, targetPosition, 0.65, 1.55, 1.4);
            profileExecutor = new StaticProfileExecutor(profile, this::output, this::getPosition, 0.02);
            profileExecutor.initialize();
        }
    }

    public double getPosition() {
        double raw = (double) pivot.getSelectedSensorPosition();
        currentPosition = Utils.lerp(raw, sensorBounds.min(), sensorBounds.max(), 0.0, 1.0);
        return currentPosition;
    }

    public void setPosition(double position) {
        double raw = Utils.lerp(position, 0.0, 1.0, sensorBounds.min(), sensorBounds.max());
        pivot.setSelectedSensorPosition((int) raw);
    }

    public double getVelocity() {
        double raw = (double) pivot.getSelectedSensorVelocity();
        return Utils.lerp(raw, -sensorBounds.size(), sensorBounds.size(), -1.0, 1.0);
    }

    public void zeroSensor() {
        currentPosition = -0.02;
        pivot.setSelectedSensorPosition((int) Utils.lerp(-0.02, 0.0, 1.0, sensorBounds.min(), sensorBounds.max()));
    }

    public void updateSensor() {
        currentPosition = getPosition();
    }

    private double getRealAngle(Double position) {
        return Utils.lerp(position, 0.346, verticalPosition, 0, 0.5 * Math.PI);
    }

    public void update() {
        profileExecutor.updateNoEnd();
    }

    public void resetPID() {
        pivot.setIntegralAccumulator(0.0, 0, 10);
    }

    private void output(Setpoint sp) {
        double lerp = Utils.lerp(sp.getPosition(), 0.0, 1.0, sensorBounds.min(), sensorBounds.max());
        double gravityCompensation = 0.15 * Math.cos(getRealAngle(currentPosition));
        pivot.set(ControlMode.Position, lerp, DemandType.ArbitraryFeedForward,
                gravityCompensation + (kF * sp.getVelocity()));
    }
}
