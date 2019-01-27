package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.pidf.Gains;
import frc.robot.pidf.PIDF;
import frc.robot.utils.Bounds;

public class Arm {
    public class Target {
        public static final double ANGLED_DOWN = 0;
        public static final double ANGLED_UP = 1 * 4096;
        public static final double UP_FLAT = 5 * 4096;
        public static final double DOWN_FLAT = 3 * 4096;
    }

    private TalonSRX pivot;

    private double startTime;
    private PIDF pid;
    private double targetPosition;

    public Arm(TalonSRX pivotMotor) {
        this.pivot = pivotMotor;

        Bounds bounds = new Bounds(-0.4, 0.4);
        Gains gains = new Gains(0.005, 0.0, 0.0);
        pid = new PIDF(gains, bounds);

        targetPosition = Target.DOWN_FLAT;
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