package frc.robot.superstructure;

public class Pose {
    public final double elevator;
    public final double arm;
    public final double ballIntake;

    Pose(double elevator, double arm, double ballIntake) {
        this.elevator = elevator;
        this.arm = arm;
        this.ballIntake = ballIntake;
    }

    public Pose setArm(double position) {
        return new Pose(elevator, position, ballIntake);
    }

    public Pose setArmMin(double position) {
        return setArm(Math.max(position, ballIntake));
    }

    public Pose setArmMax(double position) {
        return setArm(Math.min(position, ballIntake));
    }

    public Pose setElevator(double position) {
        return new Pose(position, arm, ballIntake);
    }

    public Pose setElevatorMin(double position) {
        return setElevator(Math.max(position, elevator));
    }

    public Pose setElevatorMax(double position) {
        return setElevator(Math.min(position, elevator));
    }

    public Pose setBallIntake(double position) {
        return new Pose(elevator, arm, position);
    }

    public Pose setBallIntakeMax(double position) {
        return setBallIntake(Math.min(position, ballIntake));
    }

    public Pose setBallIntakeMin(double position) {
        return setBallIntake(Math.max(position, ballIntake));
    }
}