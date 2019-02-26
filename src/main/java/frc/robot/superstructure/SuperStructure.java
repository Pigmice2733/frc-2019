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
    private Manipulator manipulator;

    public static class Target {
        // static final Pose STARTING_CONFIGURATION = new Pose(0.1,
        // Arm.Target.ANGLED_DOWN, Intake.Target.STOWED_BACK);
        // static final Pose HATCH_INTAKE = new Pose(0.1, Arm.Target.DOWN_FLAT,
        // Intake.Target.STOWED_BACK);
        // static final Pose HATCH_OUTTAKE_BOTTOM = new Pose(0.1, Arm.Target.DOWN_FLAT,
        // Intake.Target.STOWED_BACK);
        // static final Pose HATCH_OUTTAKE_MIDDLE = new Pose(0.5, Arm.Target.UP_FLAT,
        // Intake.Target.STOWED_BACK);
        // static final Pose HATCH_OUTTAKE_TOP = new Pose(1.0, Arm.Target.UP_FLAT,
        // Intake.Target.STOWED_BACK);
        // static final Pose CARGO_INTAKE = new Pose(0.5, Arm.Target.DOWN_FLAT,
        // Intake.Target.INTAKE);
        // static final Pose CARGO_OUTTAKE_BOTTOM = new Pose(0.1, Arm.Target.DOWN_FLAT,
        // Intake.Target.STOWED_UP);
        // static final Pose CARGO_OUTTAKE_MIDDLE = new Pose(0.6, Arm.Target.UP_FLAT,
        // Intake.Target.STOWED_UP);
        // static final Pose CARGO_OUTTAKE_TOP = new Pose(1, Arm.Target.ANGLED_UP,
        // Intake.Target.STOWED_UP);
        // static final Pose CARGO_OUTTAKE_SHIP = new Pose(0.6, Arm.Target.DOWN_FLAT,
        // Intake.Target.STOWED_UP);
    }

    public SuperStructure(Elevator elevator, Arm arm, Manipulator manipulator) {
        this.elevator = elevator;
        this.arm = arm;
        this.manipulator = manipulator;

        currentPose = getPose();
    }

    public void setTarget(Pose target) {
        finalTarget = target;
        currentPose = getPose();
        currentTarget = getIntermediatePose();

        elevator.setTargetPosition(currentTarget.elevator);
        arm.setTargetPosition(currentTarget.arm);

        elevator.update();
        arm.update();
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
        final Pose start = currentPose;
        final Pose end = finalTarget;

        // Prevent arm scoop from hitting battery in starting config
        if (arm.getPosition() < 0.1) {
            return end.setElevatorMin(0.3);
        }

        return end;
    }
}