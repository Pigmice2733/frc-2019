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
    private Pose currentPose;

    private Elevator elevator;
    private Intake intake;
    private Arm arm;
    private Manipulator lobster;

    public static class Target {
        static final Pose STARTING_CONFIGURATION = new Pose(0.1, Arm.Target.ANGLED_DOWN, Intake.Target.STOWED_BACK);
        static final Pose HATCH_INTAKE = new Pose(0.1, Arm.Target.DOWN_FLAT, Intake.Target.STOWED_BACK);
        static final Pose HATCH_OUTTAKE_BOTTOM = new Pose(0.1, Arm.Target.DOWN_FLAT, Intake.Target.STOWED_BACK);
        static final Pose HATCH_OUTTAKE_MIDDLE = new Pose(0.5, Arm.Target.UP_FLAT, Intake.Target.STOWED_BACK);
        static final Pose HATCH_OUTTAKE_TOP = new Pose(1.0, Arm.Target.UP_FLAT, Intake.Target.STOWED_BACK);
        static final Pose CARGO_INTAKE = new Pose(0.5, Arm.Target.DOWN_FLAT, Intake.Target.INTAKE);
        static final Pose CARGO_OUTTAKE_BOTTOM = new Pose(0.1, Arm.Target.DOWN_FLAT, Intake.Target.STOWED_UP);
        static final Pose CARGO_OUTTAKE_MIDDLE = new Pose(0.6, Arm.Target.UP_FLAT, Intake.Target.STOWED_UP);
        static final Pose CARGO_OUTTAKE_TOP = new Pose(1, Arm.Target.ANGLED_UP, Intake.Target.STOWED_UP);
        static final Pose CARGO_OUTTAKE_SHIP = new Pose(0.6, Arm.Target.DOWN_FLAT, Intake.Target.STOWED_UP);
    }

    public SuperStructure(Elevator elevator, Intake ballIntake, Arm arm) {
        this.elevator = elevator;
        this.intake = ballIntake;
        this.arm = arm;
    }

    public void set(Pose target) {
        this.finalTarget = target;
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
        // arm path goes through ball intake path
        final boolean ballIntakeHitsArm = crosses(start.arm, end.arm, new Bounds(2, 12))
                && crosses(start.ballIntake, end.ballIntake, new Bounds(5, 10));

        if (ballIntakeHitsArm) {
            return end.setBallIntake(start.ballIntake).setElevatorMin(0.5);
        }

        // hatch on arm path goes through elevator
        final boolean hatchHitsElevator = lobster.hasHatch() && crosses(start.arm, end.arm, new Bounds(10, 12));
        if (hatchHitsElevator) {
            // lift the elevator high enough to not hit the hatch, and stop the arm before
            // it hits (depends on the direction the arm is going)
            return end.setElevatorMin(0.7).setArm(end.arm - start.arm > 0 ? 0.9 : 0.7);
        }

        return null;
    }
}