package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Manipulator {
    private DoubleSolenoid solenoid1;
    private DoubleSolenoid solenoid2;
    private State lastValue = State.Retract;
    private double lastExtendedStart = 0;

    private Value solenoid1State, solenoid2State;

    public Manipulator(DoubleSolenoid solenoid1, DoubleSolenoid solendoid2) {
        this.solenoid1 = solenoid1;
        this.solenoid2 = solendoid2;

        solenoid1State = solenoid1.get();
        solenoid2State = solendoid2.get();
    }

    public enum State {
        Extend, Retract, Slack
    }

    public boolean hasHatch() {
        return true;
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