package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.controls.ButtonDebouncer;
import frc.robot.controls.SubstateToggle;
import frc.robot.controls.Toggle;
import frc.robot.superstructure.Pose;
import frc.robot.superstructure.SuperStructure;

public class ControlScheme {
    Joystick driver = new Joystick(0);
    Joystick operator = new Joystick(1);

    private Toggle hatchMode = new Toggle(operator, 9, true);
    private Toggle climbMode = new Toggle(operator, 10);

    private SubstateToggle shipToggle = new SubstateToggle(new ButtonDebouncer(operator, 3));
    private SubstateToggle cargoTopToggle = new SubstateToggle(new ButtonDebouncer(operator, 4));

    private boolean panicMode = false;
    private boolean cargoBottom = false;
    private boolean visionEngaged = false;

    public void initialize() {
        shipToggle.exit();
        cargoTopToggle.exit();

        hatchMode.set(true);
        climbMode.set(false);

        visionEngaged = false;
        cargoBottom = false;
    }

    public void update() {
        hatchMode.update();
        climbMode.update();

        shipToggle.update();
        cargoTopToggle.update();

        if (hatchMode.get() || climbMode.get()) {
            shipToggle.exit();
            cargoTopToggle.exit();
            cargoBottom = false;
        }

        if (driver.getRawButton(8) || driver.getRawButton(7)) {
            panicMode = true;
        }

        visionEngaged = driver.getRawButton(1) && driver.getY() < 0.2;
    }

    public double steer() {
        return driver.getX();
    }

    public double drive() {
        return -driver.getY();
    }

    public boolean A() {
        return operator.getRawButton(1);
    }

    public boolean B() {
        return operator.getRawButton(2);
    }

    public boolean X() {
        return operator.getRawButton(3);
    }

    public boolean Y() {
        return operator.getRawButton(4);
    }

    public boolean visionEngaged() {
        return visionEngaged;
    }

    public boolean hatchMode() {
        return hatchMode.get() && !climbMode.get();
    }

    public boolean climbMode() {
        return climbMode.get();
    }

    public boolean panicMode() {
        return panicMode;
    }

    public boolean trimMode() {
        return operator.getRawButton(8);
    }

    public boolean leftBumper() {
        return operator.getRawButton(5);
    }

    public boolean rightBumper() {
        return operator.getRawButton(6);
    }

    public Pose findSetpoint() {
        if (operator.getRawButton(7)) {
            shipToggle.exit();
            cargoTopToggle.exit();
            cargoBottom = false;
            return SuperStructure.Target.STARTING_CONFIG;
        }

        if (hatchMode()) {
            if (A()) {
                return SuperStructure.Target.HATCH_BOTTOM;
            } else if (B()) {
                return SuperStructure.Target.HATCH_M_BACK;
            } else if (X()) {
                return SuperStructure.Target.HATCH_M_FRONT;
            } else if (Y()) {
                return SuperStructure.Target.HATCH_TOP;
            }
        } else if (!climbMode()) {
            if (A()) {
                shipToggle.exit();
                cargoTopToggle.exit();
                cargoBottom = true;
                return SuperStructure.Target.CARGO_BOTTOM;
            } else if (B()) {
                shipToggle.exit();
                cargoTopToggle.exit();
                cargoBottom = false;
                return SuperStructure.Target.CARGO_OUTTAKE_BOTTOM;
            }

            if (cargoTopToggle.isEnabled()) {
                cargoBottom = false;
                shipToggle.exit();
                if (cargoTopToggle.isToggled()) {
                    return SuperStructure.Target.CARGO_B_TOP;
                } else {
                    return SuperStructure.Target.CARGO_F_TOP;
                }
            }

            if (shipToggle.isEnabled()) {
                cargoBottom = false;
                cargoTopToggle.exit();
                if (shipToggle.isToggled()) {
                    return SuperStructure.Target.CARGO_SHIP;
                } else {
                    return SuperStructure.Target.CARGO_M_FRONT;
                }
            }

            if (cargoBottom) {
                if (rightBumper()) {
                    if (operator.getRawAxis(3) > 0.7) {
                        return SuperStructure.Target.CARGO_INTAKE_HIGH;
                    } else {
                        return SuperStructure.Target.CARGO_INTAKE;
                    }
                } else {
                    return SuperStructure.Target.CARGO_BOTTOM;
                }
            }
        }

        return null;
    }
}
