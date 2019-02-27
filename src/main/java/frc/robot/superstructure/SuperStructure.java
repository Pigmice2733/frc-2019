package frc.robot.superstructure;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
import frc.robot.utils.Bounds;

public class SuperStructure {
    // end position of robot
    private Pose finalTarget;
    // current intermediate position the robot is trying to get to
    private Pose currentTarget;
    // current robot pose
    private Pose currentPose;

    private Elevator elevator;
    private Arm arm;
    private Intake intake;
    private Manipulator manipulator;

    public static class Target {
        public static final Pose STARTING_CONFIG = new Pose(0.1, Arm.Target.START, Intake.Target.STOWED_BACK);
        public static final Pose HATCH_BOTTOM = new Pose(0.0, Arm.Target.DOWN_FLAT, Intake.Target.STOWED_BACK);
        public static final Pose HATCH_MIDDLE_FRONT = new Pose(1.1, Arm.Target.DOWN_FLAT, Intake.Target.STOWED_BACK);
        public static final Pose HATCH_MIDDLE_BACK = new Pose(0.08, Arm.Target.UP_FLAT, Intake.Target.STOWED_BACK);
        public static final Pose HATCH_TOP = new Pose(0.9, Arm.Target.UP_FLAT, Intake.Target.STOWED_BACK);
        public static final Pose CARGO_BOTTOM = new Pose(0.1, Arm.Target.DOWN_FLAT, Intake.Target.INTAKE);
        public static final Pose CARGO_MIDDLE = new Pose(1.1, Arm.Target.DOWN_FLAT, Intake.Target.STOWED_UP);
        public static final Pose CARGO_TOP = new Pose(1.1, Arm.Target.UP_FLAT, Intake.Target.STOWED_UP);
    }

    public SuperStructure(Elevator elevator, Arm arm, Intake intake, Manipulator manipulator) {
        this.elevator = elevator;
        this.arm = arm;
        this.intake = intake;
        this.manipulator = manipulator;

        currentPose = getPose();
    }

    public void initialize(Pose target) {
        elevator.resetPID();
        arm.resetPID();
        intake.resetPID();

        setTarget(target);
    }

    public void setTarget(Pose target) {
        finalTarget = target;
        currentPose = getPose();
        currentTarget = getIntermediatePose();

        elevator.setTargetPosition(currentTarget.elevator);
        arm.setTargetPosition(currentTarget.arm);
        intake.setTargetPosition(currentTarget.intake);

        elevator.update();
        arm.update();
        intake.update();
    }

    public Pose getPose() {
        return new Pose(elevator.getPosition(), arm.getPosition(), 0.0);
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
    public Pose getIntermediatePose() {

        // Prevent arm scoop from hitting battery going to/from starting config
        if ((currentPose.arm < 0.05 || finalTarget.arm < 0.05)) {
            // If arm is close to start, and start is the target, let elevator down
            if (currentPose.arm < 0.0 && finalTarget.arm == Arm.Target.START) {
                return finalTarget;
                // If elevator is too low, raise immediatly
            } else if (currentPose.elevator < 0.2) {
                return currentPose.setElevatorMin(0.25);
                // Maintained raised elevator throughout move until not over battery
            } else {
                return finalTarget.setElevatorMin(0.25);
            }
        }

        return finalTarget;
    }
}
