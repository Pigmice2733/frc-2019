package frc.robot.superstructure;

public class Pose {
    public final double elevator;
    public final double arm;
    public final double intake;
    public final boolean stingers;

    Pose(double elevator, double arm, double intake, boolean stingers) {
        this.elevator = elevator;
        this.arm = arm;
        this.intake = intake;
        this.stingers = stingers;
    }

    public Pose setArm(double armPosition) {
        return new Pose(elevator, armPosition, intake, stingers);
    }

    public Pose setArmMin(double armPosition) {
        return setArm(Math.max(armPosition, intake));
    }

    public Pose setArmMax(double armPosition) {
        return setArm(Math.min(armPosition, intake));
    }

    public Pose setElevator(double elevatorPosition) {
        return new Pose(elevatorPosition, arm, intake, stingers);
    }

    public Pose setElevatorMin(double elevatorPosition) {
        return setElevator(Math.max(elevatorPosition, elevator));
    }

    public Pose setElevatorMax(double elevatorPosition) {
        return setElevator(Math.min(elevatorPosition, elevator));
    }

    public Pose setIntake(double intakePosition) {
        return new Pose(elevator, arm, intakePosition, stingers);
    }

    public Pose setIntakeMax(double intakePosition) {
        return setIntake(Math.min(intakePosition, intake));
    }

    public Pose setIntakeMin(double intakePosition) {
        return setIntake(Math.max(intakePosition, intake));
    }
}