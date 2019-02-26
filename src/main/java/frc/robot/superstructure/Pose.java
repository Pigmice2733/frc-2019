package frc.robot.superstructure;

public class Pose {
    public final double elevator;
    public final double arm;
    public final double intake;

    public Pose(double elevator, double arm, double intake) {
        this.elevator = elevator;
        this.arm = arm;
        this.intake = intake;
    }

    public Pose setArm(double position) {
        return new Pose(elevator, position, intake);
    }

    public Pose setArmMin(double position) {
        return setArm(Math.max(position, intake));
    }

    public Pose setArmMax(double position) {
        return setArm(Math.min(position, intake));
    }

    public Pose setElevator(double position) {
        return new Pose(position, arm, intake);
    }

    public Pose setElevatorMin(double position) {
        return setElevator(Math.max(position, elevator));
    }

    public Pose setElevatorMax(double position) {
        return setElevator(Math.min(position, elevator));
    }

    public Pose setIntake(double position) {
        return new Pose(elevator, arm, position);
    }

    public Pose setIntakeMax(double position) {
        return setIntake(Math.min(position, intake));
    }

    public Pose setIntakeMin(double position) {
        return setIntake(Math.max(position, intake));
    }
}