package com.pigmice.frc.robot.superstructure;

public class Pose {
    public final double elevator;
    public final double arm;
    public final double intake;

    Pose(double elevator, double arm, double intake) {
        this.elevator = elevator;
        this.arm = arm;
        this.intake = intake;
    }

    public Pose setArm(double armPosition) {
        return new Pose(elevator, armPosition, intake);
    }

    public Pose setArmMin(double armPosition) {
        return setArm(Math.max(armPosition, arm));
    }

    public Pose setArmMax(double armPosition) {
        return setArm(Math.min(armPosition, arm));
    }

    public Pose setElevator(double elevatorPosition) {
        return new Pose(elevatorPosition, arm, intake);
    }

    public Pose setElevatorMin(double elevatorPosition) {
        return setElevator(Math.max(elevatorPosition, elevator));
    }

    public Pose setElevatorMax(double elevatorPosition) {
        return setElevator(Math.min(elevatorPosition, elevator));
    }

    public Pose setIntake(double intakePosition) {
        return new Pose(elevator, arm, intakePosition);
    }

    public Pose setIntakeMax(double intakePosition) {
        return setIntake(Math.min(intakePosition, intake));
    }

    public Pose setIntakeMin(double intakePosition) {
        return setIntake(Math.max(intakePosition, intake));
    }
}
