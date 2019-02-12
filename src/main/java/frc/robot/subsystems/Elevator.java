package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.motion.Setpoint;
import frc.robot.motion.StaticProfile;
import frc.robot.motion.execution.StaticProfileExecutor;
import frc.robot.utils.Bounds;
import frc.robot.utils.NTStreamer;
import frc.robot.utils.Utils;

public class Elevator {
    private TalonSRX winch;
    private DigitalInput limitSwitch;

    private double targetPosition;

    private Bounds positionBounds = new Bounds(0.0, 1.0);

    // physical max: 30100
    // physical min: 0
    private Bounds sensorBounds = new Bounds(0, 30300.0);

    private StaticProfileExecutor profileExecutor;
    private NTStreamer<Double> positionStreamer;
    private NTStreamer<Double> targetStreamer;
    private NTStreamer<Double> setpointStreamer;
    private NTStreamer<Double> outputStreamer;

    public Elevator(TalonSRX winchMotor/*, DigitalInput limitSwitch*/) {
        this.winch = winchMotor;
        // this.limitSwitch = limitSwitch;
        winchMotor.config_kP(0, -0.01);
        winchMotor.config_kI(0, 0);
        winchMotor.config_kD(0, 0);

        targetPosition = positionBounds.min();
        positionStreamer = new NTStreamer<>("elevator", "position");
        targetStreamer = new NTStreamer<>("elevator", "target");
        setpointStreamer = new NTStreamer<>("elevator", "setpoint");
        outputStreamer = new NTStreamer<>("elevator", "output");
    }

    public void init() {
        setTargetPosition(getPosition());
    }

    public void setTargetPosition(double targetPosition) {
        if (this.targetPosition != targetPosition) {
            this.targetPosition = targetPosition;
            System.out.println("creating profile: " + targetPosition + " from " + getPosition());
            StaticProfile profile = new StaticProfile(getVelocity(), getPosition(), targetPosition, 0.1, 0.2, 0.2);
            profileExecutor = new StaticProfileExecutor(profile, this::output, Timer::getFPGATimestamp, 0.02);
        }
    }

    public double getPosition() {
        double raw = (double) winch.getSelectedSensorPosition();
        return Utils.lerp(raw, sensorBounds.min(), sensorBounds.max(), 0.0, 1.0);
    }

    public double getVelocity() {
        double raw = (double) winch.getSelectedSensorVelocity();
        return Utils.lerp(raw, sensorBounds.min(), sensorBounds.max(), -1.0, 1.0);
    }

    public void zeroSensor() {
        winch.setSelectedSensorPosition((int) sensorBounds.min());
    }

    public void updateSensor() {
        // if (limitSwitch.get()) {
        //     zeroSensor();
        // }

        positionStreamer.send(getPosition());
        targetStreamer.send(this.targetPosition);
    }

    public void update() {
        updateSensor();
        profileExecutor.update();
    }

    private void output(Setpoint sp) {
        setpointStreamer.send(sp.getPosition());
        outputStreamer.send(winch.getMotorOutputPercent());
        winch.set(ControlMode.Position, sp.getPosition(), DemandType.ArbitraryFeedForward, sp.getVelocity());
    }
}