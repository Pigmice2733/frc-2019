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

    private static final Bounds intakeCollision = new Bounds(0.115, 0.54);

    public static class Target {
        public static final Pose STARTING_CONFIG = new Pose(0.138, Arm.Target.START, Intake.Target.START);
        public static final Pose HATCH_BOTTOM = new Pose(0.0, Arm.Target.DOWN_ANGLE, Intake.Target.STOWED_BACK);
        public static final Pose HATCH_M_FRONT = new Pose(1.008, Arm.Target.DOWN_FLAT, Intake.Target.STOWED_BACK);
        public static final Pose HATCH_M_BACK = new Pose(0.0, Arm.Target.UP_ANGLE, Intake.Target.STOWED_BACK);
        public static final Pose HATCH_TOP = new Pose(0.875, Arm.Target.UP_FLAT, Intake.Target.STOWED_BACK);
        public static final Pose CARGO_BOTTOM = new Pose(0.1, Arm.Target.DOWN_ANGLE, Intake.Target.STOWED_UP);
        public static final Pose CARGO_M_FRONT = new Pose(1.0, Arm.Target.DOWN_FLAT, Intake.Target.STOWED_UP);
        public static final Pose CARGO_M_BACK = new Pose(0.0, Arm.Target.UP_ANGLE, Intake.Target.STOWED_UP);
        public static final Pose CARGO_TOP = new Pose(1.0, Arm.Target.UP_FLAT, Intake.Target.STOWED_UP);
        public static final Pose CARGO_INTAKE = new Pose(0.0, Arm.Target.DOWN_UP, Intake.Target.INTAKE);
        public static final Pose PRE_CLIMB = new Pose(0.1, Arm.Target.DOWN_ANGLE, Intake.Target.STOWED_UP);
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
    public Pose getIntermediatePose(Pose current, Pose target) {

        // Prevent ball intake from swinging through bottom of arm
        if (crosses(current.intake, target.intake, intakeCollision)) {
            // If ball intake ends vertical, raise elevator, lower intake, get elevator to
            // right spot, then let everything go to final intake (which raises intake to
            // vertical)
            if (intakeCollision.contains(target.intake)
                    && (!intakeCollision.contains(current.intake) || current.elevator > 0.9 || current.arm > 0.5)) {
                if (current.intake < 0.54 && (current.elevator > 0.14 || current.intake < 0.3)) {
                    // raise elevator
                    if (current.elevator < 0.9) {
                        return current.setElevatorMin(1.0).setArm(Arm.Target.DOWN_ANGLE);
                        // keep elevator up, lower intake
                    } else {
                        return current.setElevatorMin(1.0).setArm(Arm.Target.DOWN_ANGLE).setIntakeMin(0.56);
                    }
                    // move elevator to right spot while keeping intake down
                } else if (current.elevator > 0.13 || current.arm > 0.05) {
                    return target.setIntakeMin(0.56);
                    // move everything to final target
                } else {
                    return target;
                }
                // If ball intake is vertical and over top of hatch intake, lower ball intake
            } else if (intakeCollision.contains(current.intake) && current.elevator < 0.8) {
                return current.setIntakeMin(0.56);
                // If ball intake isn't over hatch intake, raise elevator first then move to
                // final location
            } else {
                if (current.elevator < 0.9) {
                    return current.setElevatorMin(1.0).setArmMin(Arm.Target.DOWN_ANGLE);
                } else {
                    return target.setElevatorMin(1.0).setArmMin(Arm.Target.DOWN_ANGLE);
                }
            }
        }

        // Prevent arm scoop from hitting battery going to/from starting config
        if ((current.arm < 0.11 || target.arm < 0.11)) {
            // If arm is close to start, and start is the target, let elevator down
            if (current.arm < 0.0 && Utils.almostEquals(target.arm, Arm.Target.START)) {
                return target;
                // If elevator is too low, raise immediatly
            } else if (current.elevator < 0.175 && current.arm < 0.08) {
                return current.setElevatorMin(0.2);
                // Maintained raised elevator throughout move until not over battery
            } else {
                if (current.arm > 0.08) {
                    return target.setElevatorMin(0.0);
                } else {
                    return target.setElevatorMin(0.2);
                }
            }
        }

        return target;
    }
}
