package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Stingers {
    private DoubleSolenoid leftSolenoid, rightSolenoid;

    public Stingers(DoubleSolenoid leftSolenoid, DoubleSolenoid rightSolenoid) {
        this.leftSolenoid = leftSolenoid;
        this.rightSolenoid = rightSolenoid;
    }

    public boolean isExtending() {
        return leftSolenoid.get().equals(Value.kReverse);
    }

    public void fire() {
        leftSolenoid.set(Value.kReverse);
        rightSolenoid.set(Value.kReverse);
    }

    public void retract() {
        leftSolenoid.set(Value.kForward);
        rightSolenoid.set(Value.kForward);
    }
}