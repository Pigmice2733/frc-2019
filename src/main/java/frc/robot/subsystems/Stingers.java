package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Stingers {
    private DoubleSolenoid solenoid;

    public Stingers(DoubleSolenoid solenoid) {
        this.solenoid = solenoid;
    }

    public void fire() {
        solenoid.set(Value.kReverse);
    }

    public void stop() {
        solenoid.set(Value.kOff);
    }

    public void retract() {
        solenoid.set(Value.kForward);
    }
}