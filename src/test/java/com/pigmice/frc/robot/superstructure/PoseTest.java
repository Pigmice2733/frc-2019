package com.pigmice.frc.robot.superstructure;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

public class PoseTest {
    private static final double epsilon = 1e-6;

    @Test
    public void positions() {
        Pose pose = new Pose(0.0, 1.0, 2.0);

        Assertions.assertEquals(0.0, pose.elevator, epsilon);
        Assertions.assertEquals(1.0, pose.arm, epsilon);
        Assertions.assertEquals(2.0, pose.intake, epsilon);
    }

    @Test
    public void set() {
        Pose pose = new Pose(1.5, 2.5, 3.5);

        Assertions.assertEquals(0.0, pose.setElevator(0).elevator, epsilon);
        Assertions.assertEquals(0.5, pose.setArm(0.5).arm, epsilon);
        Assertions.assertEquals(1.0, pose.setIntake(1.0).intake, epsilon);
    }

    @Test
    public void setMin() {
        Pose pose = new Pose(2.0, 3.5, 4.5);

        Assertions.assertEquals(2.0, pose.setElevatorMin(1.0).elevator, epsilon);
        Assertions.assertEquals(3.5, pose.setElevatorMin(3.5).elevator, epsilon);

        Assertions.assertEquals(3.5, pose.setArmMin(2.0).arm, epsilon);
        Assertions.assertEquals(4.0, pose.setArmMin(4.0).arm, epsilon);

        Assertions.assertEquals(4.5, pose.setIntakeMin(3.0).intake, epsilon);
        Assertions.assertEquals(5.5, pose.setIntakeMin(5.5).intake, epsilon);
    }

    @Test
    public void setMax() {
        Pose pose = new Pose(1.0, 2.0, 3.0);

        Assertions.assertEquals(1.0, pose.setElevatorMax(2.0).elevator, epsilon);
        Assertions.assertEquals(0.0, pose.setElevatorMax(0.0).elevator, epsilon);

        Assertions.assertEquals(2.0, pose.setArmMax(4.0).arm, epsilon);
        Assertions.assertEquals(-2.0, pose.setArmMax(-2.0).arm, epsilon);

        Assertions.assertEquals(3.0, pose.setIntakeMax(4.5).intake, epsilon);
        Assertions.assertEquals(1.0, pose.setIntakeMax(1.0).intake, epsilon);
    }
}
