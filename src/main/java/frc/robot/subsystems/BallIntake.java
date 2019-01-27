package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.pidf.Gains;
import frc.robot.pidf.PIDF;
import frc.robot.utils.Bounds;

public class BallIntake {
    public class Target {
        public static final double INTAKE = 0;
        public static final double STOWED_UP = 3 * 4096;
        public static final double STOWED_BACK = 5 * 4096;
    }

    private TalonSRX pivot;

    private double startTime;
    private PIDF pid;
    private double targetPosition;

    public BallIntake(TalonSRX pivotMotor) {
        this.pivot = pivotMotor;

        Bounds bounds = new Bounds(-0.4, 0.4);
        Gains gains = new Gains(0.005, 0.0, 0.0);
        pid = new PIDF(gains, bounds);

        targetPosition = Target.STOWED_UP;
    }

    public void init() {
        startTime = Timer.getFPGATimestamp();
        pid.initialize((double) pivot.getSelectedSensorPosition(), 0.0, 0.0);
    }

    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
    }

    public int getPosition() {
        return pivot.getSelectedSensorPosition();
    }

    public void update() {
        int currentPosition = getPosition();
        double motorSpeed = pid.calculateOutput((double) currentPosition, (double) targetPosition,
                Timer.getFPGATimestamp() - startTime);

        pivot.set(ControlMode.PercentOutput, motorSpeed);
    }
}