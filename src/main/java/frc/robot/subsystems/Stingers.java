package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Stingers {
    private DoubleSolenoid solenoid1, solenoid2;

    public Stingers(DoubleSolenoid solenoid1, DoubleSolenoid solenoid2) {
        this.solenoid1 = solenoid1;
        this.solenoid2 = solenoid2;
    }

    public boolean isExtending() {
        return solenoid1.get().equals(Value.kReverse);
    }

    public void extend() {
        solenoid1.set(Value.kReverse);
        solenoid2.set(Value.kReverse);
    }

    public void retract() {
        solenoid1.set(Value.kForward);
        solenoid2.set(Value.kForward);
    }

    // public void stop() {
    // solenoid1.set(Value.kReverse);
    // solenoid2.set(Value.kForward);
    // }
}