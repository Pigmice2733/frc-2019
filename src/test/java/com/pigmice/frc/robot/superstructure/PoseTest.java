package com.pigmice.frc.robot.superstructure;

import org.junit.Assert;
import org.junit.Test;

public class PoseTest {
    private static final double epsilon = 1e-6;

    @Test
    public void positions() {
        Pose pose = new Pose(0.0, 1.0, 2.0);

        Assert.assertEquals(0.0, pose.elevator, epsilon);
        Assert.assertEquals(1.0, pose.arm, epsilon);
        Assert.assertEquals(2.0, pose.intake, epsilon);
    }

    @Test
    public void set() {
        Pose pose = new Pose(1.5, 2.5, 3.5);

        Assert.assertEquals(0.0, pose.setElevator(0).elevator, epsilon);
        Assert.assertEquals(0.5, pose.setArm(0.5).arm, epsilon);
        Assert.assertEquals(1.0, pose.setIntake(1.0).intake, epsilon);
    }

    @Test
    public void setMin() {
        Pose pose = new Pose(2.0, 3.5, 4.5);

        Assert.assertEquals(2.0, pose.setElevatorMin(1.0).elevator, epsilon);
        Assert.assertEquals(3.5, pose.setElevatorMin(3.5).elevator, epsilon);

        Assert.assertEquals(3.5, pose.setArmMin(2.0).arm, epsilon);
        Assert.assertEquals(4.0, pose.setArmMin(4.0).arm, epsilon);

        Assert.assertEquals(4.5, pose.setIntakeMin(3.0).intake, epsilon);
        Assert.assertEquals(5.5, pose.setIntakeMin(5.5).intake, epsilon);
    }

    @Test
    public void setMax() {
        Pose pose = new Pose(1.0, 2.0, 3.0);

        Assert.assertEquals(1.0, pose.setElevatorMax(2.0).elevator, epsilon);
        Assert.assertEquals(0.0, pose.setElevatorMax(0.0).elevator, epsilon);

        Assert.assertEquals(2.0, pose.setArmMax(4.0).arm, epsilon);
        Assert.assertEquals(-2.0, pose.setArmMax(-2.0).arm, epsilon);

        Assert.assertEquals(3.0, pose.setIntakeMax(4.5).intake, epsilon);
        Assert.assertEquals(1.0, pose.setIntakeMax(1.0).intake, epsilon);
    }
}
