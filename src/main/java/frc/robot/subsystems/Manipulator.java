package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Manipulator {
    private DoubleSolenoid piston1;
    private DoubleSolenoid piston2;
    private State lastValue = State.Retract;
    private double lastExtendedStart = 0;

    public Manipulator(DoubleSolenoid piston1, DoubleSolenoid piston2) {
        this.piston1 = piston1;
        this.piston2 = piston2;
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
        piston1.set(Value.kReverse);
        piston2.set(Value.kReverse);
        lastValue = State.Slack;
    }

    private void retract() {
        piston1.set(Value.kReverse);
        piston2.set(Value.kForward);
        lastValue = State.Retract;
    }

    private void extend() {
        piston1.set(Value.kForward);
        piston2.set(Value.kReverse);
        lastValue = State.Extend;
    }
}