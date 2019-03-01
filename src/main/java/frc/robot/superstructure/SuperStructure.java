package frc.robot.superstructure;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.pidf.Gains;
import frc.robot.pidf.PIDF;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Stingers;
import frc.robot.utils.Bounds;
import frc.robot.utils.Utils;

public class SuperStructure {
    // // end position of robot
    // private Pose finalTarget;
    // // current intermediate position the robot is trying to get to
    // private Pose currentTarget;
    // // current robot pose
    // private Pose currentPose;

    private Elevator elevator;
    private Arm arm;
    private Intake intake;
    private Stingers stingers;

    private AHRS navx;

    private PIDF intakeBalancer;

    private static final Bounds intakeCollision = new Bounds(0.1, 0.5);

    public static class Target {
        public static final Pose STARTING_CONFIG = new Pose(0.1, Arm.Target.START, Intake.Target.STOWED_BACK, false);
        public static final Pose HATCH_BOTTOM = new Pose(0.0, Arm.Target.DOWN_FLAT, Intake.Target.STOWED_BACK, false);
        public static final Pose HATCH_M_FRONT = new Pose(1.1, Arm.Target.DOWN_FLAT, Intake.Target.STOWED_BACK, false);
        public static final Pose HATCH_M_BACK = new Pose(0.08, Arm.Target.UP_FLAT, Intake.Target.STOWED_BACK, false);
        public static final Pose HATCH_TOP = new Pose(0.9, Arm.Target.UP_FLAT, Intake.Target.STOWED_BACK, false);
        public static final Pose CARGO_BOTTOM = new Pose(0.1, Arm.Target.DOWN_FLAT, Intake.Target.INTAKE, false);
        public static final Pose CARGO_MIDDLE = new Pose(1.1, Arm.Target.DOWN_FLAT, Intake.Target.STOWED_UP, false);
        public static final Pose CARGO_TOP = new Pose(1.1, Arm.Target.UP_FLAT, Intake.Target.STOWED_UP, false);
    }

    public SuperStructure(Elevator elevator, Arm arm, Intake intake, Stingers stingers, AHRS navx) {
        this.elevator = elevator;
        this.arm = arm;
        this.intake = intake;

        this.navx = navx;

        Gains balancerGains = new Gains(0.001, 0.0, 0.0);
        Bounds balancerOutput = new Bounds(-0.25, 0.25);
        intakeBalancer = new PIDF(balancerGains, balancerOutput);
    }

    public void initialize(Pose target) {
        elevator.resetPID();
        arm.resetPID();
        // intake.resetPID();

        setTarget(target);
    }

    public void setTarget(Pose target) {
        Pose currentPose = getPose();
        Pose currentTarget = getIntermediatePose(currentPose, target);

        elevator.setTargetPosition(currentTarget.elevator);
        arm.setTargetPosition(currentTarget.arm);

        elevator.update();
        arm.update();

        // if (target.stingers) {
        // if (currentPose.intake > 0.5) {
        // if (!stingers.isExtending()) {
        // intakeBalancer.initialize(0.0, Timer.getFPGATimestamp(), 0.0);
        // }
        // stingers.fire();

        // double intakeOutput = intakeBalancer.calculateOutput(navx.getPitch(), 0.0,
        // Timer.getFPGATimestamp());
        // if (currentPose.intake < 0.05 || currentPose.intake > 0.95) {
        // intake.drive(0.0);
        // } else {
        // intake.drive(intakeOutput);
        // }
        // } else {
        // stingers.retract();
        // }
        // } else {
        // intake.setTargetPosition(currentTarget.intake);
        // intake.update();
        // stingers.retract();
        // }
    }

    public Pose getPose() {
        return new Pose(elevator.getPosition(), arm.getPosition(), 0.0, false);
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

        // Prevent arm scoop from hitting battery going to/from starting config
        if ((current.arm < 0.05 || target.arm < 0.05)) {
            // If arm is close to start, and start is the target, let elevator down
            if (current.arm < 0.0 && Utils.almostEquals(target.arm, Arm.Target.START)) {
                return target;
                // If elevator is too low, raise immediatly
            } else if (current.elevator < 0.2) {
                return current.setElevatorMin(0.25);
                // Maintained raised elevator throughout move until not over battery
            } else {
                return target.setElevatorMin(0.25);
            }
        }

        // Prevent ball intake from swinging through bottom of arm
        if (crosses(current.intake, target.intake, intakeCollision)) {
            if (current.elevator < 0.6) {
                return current.setElevatorMin(0.8).setArmMin(0.1);
            } else {
                return target.setElevatorMin(0.8).setArmMin(0.1);
            }
        }

        return target;
    }
}
