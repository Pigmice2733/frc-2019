package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Manipulator {
    private DoubleSolenoid piston;

    public boolean hasHatch() {
        return true;
    }

    public Manipulator(DoubleSolenoid piston) {
        this.piston = piston;
    }

    public void setPosition(boolean open) {
        if (open) {
            piston.set(Value.kForward);
        } else {
            piston.set(Value.kReverse);
        }
    }
}