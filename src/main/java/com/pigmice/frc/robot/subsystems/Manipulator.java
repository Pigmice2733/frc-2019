package com.pigmice.frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Timer;

public class Manipulator {
    private VictorSPX motor;

    private AnalogInput IR;

    private DoubleSolenoid solenoid1;
    private DoubleSolenoid solenoid2;
    private State lastValue = State.Retract;
    private double lastExtendedStart = 0;

    private Value solenoid1State, solenoid2State;

    private final int detectionThreshold = 815;

    public Manipulator(DoubleSolenoid solenoid1, DoubleSolenoid solendoid2, VictorSPX motor, AnalogInput IR) {
        this.solenoid1 = solenoid1;
        this.solenoid2 = solendoid2;

        this.motor = motor;

        this.IR = IR;
        this.IR.setAverageBits(2);
        this.IR.setOversampleBits(0);

        solenoid1State = solenoid1.get();
        solenoid2State = solendoid2.get();
    }

    public enum State {
        Extend, Retract, Slack
    }

    public boolean hasBall() {
        return IR.getAverageValue() > detectionThreshold;
    }

    public void setPosition(State value) {
        switch (value) {
        case Retract:
            retract();
            break;
        case Extend:
            extend();
            break;
        default:
            if (lastValue == State.Extend) {
                // if it has finished extending
                if (Timer.getFPGATimestamp() - lastExtendedStart > 0.3) {
                    slack();
                }
            } else if (lastValue == State.Retract) {
                extend();
                lastExtendedStart = Timer.getFPGATimestamp();
            }
            break;
        }
    }

    public void drive(double speed) {
        motor.set(ControlMode.PercentOutput, speed);
    }

    private void slack() {
        update(Value.kReverse, Value.kReverse);
        lastValue = State.Slack;
    }

    private void retract() {
        update(Value.kReverse, Value.kForward);
        lastValue = State.Retract;
    }

    private void extend() {
        update(Value.kForward, Value.kReverse);
        lastValue = State.Extend;
    }

    private void update(Value newSolenoid1, Value newSolenoid2) {
        if (newSolenoid1 != solenoid1State) {
            solenoid1.set(newSolenoid1);
            solenoid1State = newSolenoid1;
        }

        if (newSolenoid2 != solenoid2State) {
            solenoid2.set(newSolenoid2);
            solenoid2State = newSolenoid2;
        }
    }
}
