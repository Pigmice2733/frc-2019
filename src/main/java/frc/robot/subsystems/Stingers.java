package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Stingers {
    private DoubleSolenoid mode, control;

    public Stingers(DoubleSolenoid mode, DoubleSolenoid control) {
        this.mode = mode;
        this.control = control;
    }

    public boolean isExtending() {
        return mode.get().equals(Value.kReverse);
    }

    public void extend() {
        mode.set(Value.kReverse);
        control.set(Value.kForward);
    }

    public void retract() {
        mode.set(Value.kForward);
        control.set(Value.kReverse);
    }

    public void stop() {
        mode.set(Value.kReverse);
        control.set(Value.kReverse);
    }
}