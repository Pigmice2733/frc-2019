package com.pigmice.frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Stingers {
    private DoubleSolenoid mode, control;

    private Value modeState = Value.kReverse, controlState = Value.kForward;

    public Stingers(DoubleSolenoid control, DoubleSolenoid mode) {
        this.mode = mode;
        this.control = control;
    }

    public boolean isExtending() {
        return mode.get().equals(Value.kReverse);
    }

    public void extend() {
        update(Value.kReverse, Value.kForward);
    }

    public void retract() {
        update(Value.kForward, Value.kReverse);
    }

    public void stop() {
        update(Value.kReverse, Value.kReverse);
    }

    private void update(Value newMode, Value newControl) {
        if (newMode != modeState) {
            mode.set(newMode);
            modeState = newMode;
        }

        if (newControl != controlState) {
            control.set(newControl);
            controlState = newControl;
        }
    }
}
