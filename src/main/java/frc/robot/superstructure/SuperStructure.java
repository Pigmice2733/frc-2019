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
        public static final Pose CARGO_BOTTOM = new Pose(0.1, Arm.Target.DOWN_SLIGHT, Intake.Target.STOWED_UP);
        public static final Pose CARGO_M_FRONT = new Pose(0.992, Arm.Target.DOWN_FLAT, Intake.Target.STOWED_UP);
        public static final Pose CARGO_M_BACK = new Pose(0.0, Arm.Target.UP_ANGLE, Intake.Target.STOWED_UP);
        public static final Pose CARGO_TOP = new Pose(0.992, Arm.Target.UP_FLAT, Intake.Target.STOWED_UP);
        public static final Pose CARGO_INTAKE = new Pose(0.0, Arm.Target.DOWN_UP, Intake.Target.INTAKE);
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
                if (current.arm < 0.3 && current.elevator < 0.9 && (target.arm > 0.1 || target.elevator > 0.12
                        || (current.intake > 0.2 && target.intake < 0.1))) {
                    // move the intake out in front
                    setState("A");
                    return current.setIntakeMin(0.56);
                }

                // Arm is trying to swing down
                if (target.arm < 0.3 && target.elevator < 0.9 && (current.elevator > 0.12 || current.arm > 0.1)) {
                    if (intakeCollision.contains(target.intake)) {
                        setState("B");
                        return target.setIntakeMin(0.56).setArmMin(0.4).setElevatorMin(0.35);
                    } else {
                        setState("C");
                        return target.setArmMin(0.4).setElevatorMin(0.35);
                    }
                }
            }

            if (intakeCollision.contains(target.intake) && (current.arm < 0.3 || target.arm < 0.3)
                    && (current.elevator < 0.85 || target.elevator < 0.9)) {
                if (current.intake < 0.1 && current.elevator < 0.3) {
                    setState("D");
                    return current.setElevatorMin(0.35).setArmMin(0.4);
                }

                if (current.intake < 0.54 && (current.elevator > 0.12 || current.arm > 0.1)) {
                    setState("E");
                    return current.setIntakeMin(0.56).setElevatorMin(0.35).setArmMin(0.4);
                }

                if (current.elevator > 0.12 || current.arm > 0.1 || target.arm > 0.5) {
                    setState("F");
                    return target.setIntakeMin(0.56);
                }

                setState("G");
                return target;
            }

            if (current.elevator < 0.9) {
                if (current.arm > 0.6 && target.arm > 0.6) {
                    setState("H");
                    return target;
                }

                if (current.arm > 0.35 && (current.elevator > 0.3 || target.arm > 0.4)) {
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

        // Enter starting config
        if (Utils.almostEquals(target.arm, Arm.Target.START)) {
            if (current.arm > 0.0) {
                if (current.elevator < 0.15 && current.arm < 0.1) {
                    setState("L");
                    return target.setElevatorMin(0.2).setArmMin(0.005);
                } else {
                    setState("M");
                    return target.setElevatorMin(0.2);
                }
            } else {
                if (current.elevator < 0.132) {
                    setState("N");
                    return current.setArmMin(0.005).setElevator(0.12);
                }
                setState("O");
                return target;
            }
        }

        // Exit starting config
        if (current.arm < 0.01) {
            if (current.elevator < 0.15) {
                setState("P");
                return current.setElevatorMin(0.2);
            } else {
                setState("Q");
                return target.setElevatorMin(0.2);
            }
        }

        setState("R");
        return target;
    }
}
