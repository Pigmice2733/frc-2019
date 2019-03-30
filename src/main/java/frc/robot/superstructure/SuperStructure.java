package frc.robot.superstructure;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Stingers;
import frc.robot.utils.Bounds;
import frc.robot.utils.Utils;

public class SuperStructure {
    private Elevator elevator;
    private Arm arm;
    private Intake intake;

    private Pose finalTarget;

    private static final Bounds intakeCollision = new Bounds(0.115, 0.47);

    private static String lastState = "S";

    public static void setState(String state) {
        if (!state.equals(lastState)) {
            lastState = state;
            System.out.println(lastState);
        }
    }

    public static class Target {
        public static final Pose STARTING_CONFIG = new Pose(0.138, Arm.Target.START, Intake.Target.START);
        public static final Pose HATCH_BOTTOM = new Pose(0.1, Arm.Target.DOWN_SLIGHT, Intake.Target.STOWED_BACK);
        public static final Pose HATCH_M_FRONT = new Pose(0.992, Arm.Target.DOWN_FLAT, Intake.Target.STOWED_BACK);
        public static final Pose HATCH_M_BACK = new Pose(0.0, Arm.Target.UP_ANGLE, Intake.Target.STOWED_BACK);
        public static final Pose HATCH_TOP = new Pose(0.875, Arm.Target.UP_FLAT, Intake.Target.STOWED_BACK);
        public static final Pose CARGO_BOTTOM = new Pose(0.1, Arm.Target.DOWN_SLIGHT, Intake.Target.STOWED_FRONT);
        public static final Pose CARGO_M_FRONT = new Pose(0.992, 0.27, Intake.Target.STOWED_UP);
        public static final Pose CARGO_M_BACK = new Pose(0.0, 0.97, Intake.Target.STOWED_UP);
        public static final Pose CARGO_TOP = new Pose(0.98, 0.50, Intake.Target.STOWED_UP);
        public static final Pose CARGO_INTAKE = new Pose(0.02, Arm.Target.DOWN_UP, Intake.Target.INTAKE);
        public static final Pose CARGO_INTAKE_HIGH = new Pose(0.02, Arm.Target.DOWN_UP, Intake.Target.INTAKE - 0.075);
        public static final Pose CARGO_OUTTAKE_BOTTOM = new Pose(0.0, Arm.Target.CARGO_OUTTAKE, 0.5);
        public static final Pose PRE_CLIMB = new Pose(0.1, Arm.Target.CLIMB, Intake.Target.CLIMB);
    }

    public SuperStructure(Elevator elevator, Arm arm, Intake intake, Stingers stingers, AHRS navx) {
        this.elevator = elevator;
        this.arm = arm;
        this.intake = intake;
    }

    public void initialize(Pose target) {
        elevator.resetPID();
        arm.resetPID();
        intake.resetPID();

        target(target);
    }

    public void target(Pose target) {
        finalTarget = target;
        Pose currentPose = getPose();
        Pose currentTarget = getIntermediatePose(currentPose, target);

        elevator.setTargetPosition(currentTarget.elevator);
        arm.setTargetPosition(currentTarget.arm);
        intake.setTargetPosition(currentTarget.intake);

        elevator.update();
        arm.update();
        intake.update();
    }

    public void update() {
        target(finalTarget);
    }

    public Pose getTarget() {
        return finalTarget;
    }

    public Pose getPose() {
        return new Pose(elevator.getPosition(), arm.getPosition(), intake.getPosition());
    }

    /**
     * Returns whether an intermediate value is between a start and end position
     */
    private static boolean crosses(double start, double end, Bounds boundary) {
        Bounds range = new Bounds(start, end);
        return range.overlaps(boundary);
    }

    /**
     * Returns the next intermediate step in reaching the final target
     */
    public static Pose getIntermediatePose(Pose current, Pose target) {
        // Prevent ball intake from colliding with arm manipulators
        if (crosses(current.intake, target.intake, intakeCollision)) {
            // Intake could currently be above manipulators
            if (intakeCollision.contains(current.intake)) {
                // Arm starts down
                if (current.arm < 0.3 && current.elevator < 0.9 && (target.arm > 0.075 || target.elevator > 0.12
                        || (current.intake > 0.2 && target.intake < 0.1))) {
                    // move the intake out in front
                    setState("A");
                    return current.setIntakeMin(0.56);
                }

                // Arm is trying to swing down
                if (target.arm < 0.3 && target.elevator < 0.9 && (current.elevator > 0.12 || current.arm > 0.1)) {
                    if (intakeCollision.contains(target.intake)) {
                        if (current.intake < 0.475) {
                            setState("B1");
                            return current.setIntakeMin(0.56);
                        } else {
                            setState("B2");
                            return target.setIntakeMin(0.56).setArmMin(0.4).setElevatorMin(0.35);
                        }
                    } else {
                        if (current.intake < 0.475 && target.intake > 0.2 && target.arm < 0.2) {
                            setState("C1");
                            return current.setIntakeMin(0.56);
                        } else {
                            setState("C2");
                            return target.setArmMin(0.4).setElevatorMin(0.35);
                        }
                    }
                }
            }

            if (intakeCollision.contains(target.intake) && (current.arm < 0.3 || target.arm < 0.3)
                    && (current.elevator < 0.85 || target.elevator < 0.9)
                    && !Utils.almostEquals(target.arm, Arm.Target.DOWN_UP)) {
                if (current.intake < 0.1 && current.elevator < 0.3) {
                    setState("D");
                    return target.setElevatorMin(0.35).setArmMin(0.4).setIntake(current.intake);
                }

                if (current.intake < 0.54 && (current.elevator > 0.12 || current.arm > 0.1)) {
                    if (target.elevator > 0.875) {
                        setState("E1");
                        return target.setIntakeMin(0.56).setElevatorMin(0.35).setArmMin(0.2);
                    } else {
                        if (current.intake > 0.465 && target.arm < 0.15) {
                            if (target.elevator < 0.01 && target.arm <= 0.11 && target.arm > 0.1) {
                                setState("E2");
                                return target.setIntakeMin(0.56).setElevatorMin(0.05);
                            }
                            setState("E3");
                            return target.setIntakeMin(0.56).setArmMin(current.arm);
                        } else {
                            setState("E4");
                            return target.setIntakeMin(0.56).setElevatorMin(0.35).setArmMin(0.4);
                        }
                    }
                }

                if (current.elevator > 0.12 || current.arm > 0.1 || target.arm > 0.4 || target.elevator > 0.875) {
                    setState("F");
                    return target.setIntakeMin(0.56);
                }

                setState("G");
                return target;
            }

            if (current.elevator < 0.9 && !Utils.almostEquals(target.arm, Arm.Target.DOWN_UP)) {
                if (current.arm > 0.6 && target.arm > 0.6) {
                    setState("H");
                    return target;
                }

                if ((current.arm > 0.3 || (current.intake > 0.4 && current.elevator > 0.2))
                        && (current.elevator > 0.15 || target.arm > 0.35 || target.elevator > 0.875)) {
                    if (target.arm > 0.7) {
                        setState("I");
                        return target;
                    } else {
                        setState("J");
                        return target.setElevatorMin(0.35).setArmMin(0.4);
                    }
                }

                setState("K");
                return current.setElevatorMin(0.35).setArmMin(0.4);
            }
        }

        if (target.intake >= 0.5 && Utils.almostEquals(target.arm, Arm.Target.CARGO_OUTTAKE)) {
            if (current.arm < (target.arm - 0.035)) {
                setState("L1");
                return target.setIntakeMin(0.625).setElevatorMin(0.2);
            }
        }

        if (target.intake >= 0.5 && Utils.almostEquals(target.arm, Arm.Target.DOWN_UP)) {
            if (current.arm < (target.arm - 0.025)) {
                setState("L2");
                return target.setElevatorMin(0.2);
            }
        }

        // Enter starting config
        if (Utils.almostEquals(target.arm, Arm.Target.START)) {
            if (current.arm > 0.0) {
                if (current.elevator < 0.15 && current.arm > 0.01) {
                    setState("M");
                    return target.setElevatorMin(0.2).setArmMin(0.025);
                } else {
                    setState("N");
                    return target.setElevatorMin(0.2);
                }
            } else {
                if (current.elevator < 0.13) {
                    setState("O");
                    return current.setArmMin(0.025).setElevator(0.1);
                }
                setState("P");
                return target;
            }
        }

        // Exit starting config
        if (current.arm < 0.01) {
            if (current.elevator < 0.15) {
                setState("Q");
                return current.setElevatorMin(0.2);
            } else {
                setState("R");
                return target.setElevatorMin(0.2);
            }
        }

        setState("S");
        return target;
    }
}
