package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Stingers {
    private DoubleSolenoid solenoid;

    public Stingers(DoubleSolenoid solenoid) {
        this.solenoid = solenoid;
    }

    public boolean isExtending() {
        return solenoid.get().equals(Value.kReverse);
    }

    public void fire() {
        solenoid.set(Value.kReverse);
    }

    public void retract() {
        solenoid.set(Value.kForward);
    }
}