package com.pigmice.frc.robot.superstructure;

import static org.hamcrest.MatcherAssert.assertThat;
import static org.hamcrest.Matchers.greaterThanOrEqualTo;

import com.pigmice.frc.lib.logging.Logger;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class SuperStructureTest {
    private static final double epsilon = 1e-6;

    @BeforeAll
    public static void configureLogger() {
        Logger.configure(null);
        Logger.start();
    }

    @Test
    public void startToHatch() {
        Pose current = SuperStructure.Target.STARTING_CONFIG;
        Pose target = SuperStructure.Target.HATCH_BOTTOM;

        Pose intermediate = SuperStructure.getIntermediatePose(current, target);

        assertThat("Elevator raised", intermediate.elevator, greaterThanOrEqualTo(0.2));
        Assertions.assertEquals(current.arm, intermediate.arm, epsilon);
        Assertions.assertTrue(intermediate.intake < 0.115 && intermediate.intake >= 0.0);

        current = new Pose(0.16, current.arm, 0.05);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        assertThat("Elevator raised", intermediate.elevator, greaterThanOrEqualTo(0.2));
        Assertions.assertEquals(target.arm, intermediate.arm, epsilon);
        Assertions.assertTrue(intermediate.intake < 0.115 && intermediate.intake >= 0.0);

        current = new Pose(0.16, 0.01, 0.05);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        Assertions.assertEquals(target.elevator, intermediate.elevator, epsilon);
        Assertions.assertEquals(target.arm, intermediate.arm, epsilon);
        Assertions.assertTrue(intermediate.intake < 0.115 && intermediate.intake >= 0.0);
    }

    @Test
    public void hatchTopToStart() {
        Pose current = SuperStructure.Target.HATCH_M_BACK;
        Pose target = SuperStructure.Target.STARTING_CONFIG;

        Pose intermediate = SuperStructure.getIntermediatePose(current, target);

        assertThat("Elevator raised", intermediate.elevator, greaterThanOrEqualTo(0.15));
        Assertions.assertEquals(0.02, intermediate.arm, epsilon);
        Assertions.assertTrue(intermediate.intake < 0.115 && intermediate.intake >= 0.0);

        current = new Pose(0.1, 0.001, 0.05);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        assertThat("Elevator raised", intermediate.elevator, greaterThanOrEqualTo(0.15));
        Assertions.assertEquals(target.arm, intermediate.arm, epsilon);
        Assertions.assertTrue(intermediate.intake < 0.115 && intermediate.intake >= 0.0);

        current = new Pose(0.2, 0.001, 0.05);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        assertThat("Elevator raised", intermediate.elevator, greaterThanOrEqualTo(0.15));
        Assertions.assertEquals(target.arm, intermediate.arm, epsilon);
        Assertions.assertTrue(intermediate.intake < 0.115 && intermediate.intake >= 0.0);

        current = new Pose(0.16, -0.001, 0.05);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        Assertions.assertEquals(target.elevator, intermediate.elevator, epsilon);
        Assertions.assertEquals(target.arm, intermediate.arm, epsilon);
        Assertions.assertTrue(intermediate.intake < 0.115 && intermediate.intake >= 0.0);
    }

    @Test
    public void hatchBottomToTop() {
        Pose current = SuperStructure.Target.HATCH_BOTTOM;
        Pose target = SuperStructure.Target.HATCH_TOP;

        Pose intermediate = SuperStructure.getIntermediatePose(current, target);

        Assertions.assertEquals(target.elevator, intermediate.elevator, epsilon);
        Assertions.assertEquals(target.arm, intermediate.arm, epsilon);
        Assertions.assertEquals(target.intake, intermediate.intake, epsilon);
    }

    @Test
    public void hatchToBall() {
        Pose current = SuperStructure.Target.HATCH_BOTTOM;
        Pose target = SuperStructure.Target.CARGO_BOTTOM;

        Pose intermediate = SuperStructure.getIntermediatePose(current, target);

        assertThat("Elevator raised", intermediate.elevator, greaterThanOrEqualTo(0.3));
        assertThat("Arm raised", intermediate.arm, greaterThanOrEqualTo(0.4));
        Assertions.assertEquals(current.intake, intermediate.intake, epsilon);

        current = new Pose(0.31, 0.36, current.intake);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        assertThat("Elevator raised", intermediate.elevator, greaterThanOrEqualTo(0.3));
        assertThat("Arm raised", intermediate.arm, greaterThanOrEqualTo(0.4));
        assertThat("Intake forward", intermediate.intake, greaterThanOrEqualTo(0.55));

        current = new Pose(0.975, target.arm, 0.55);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        Assertions.assertEquals(target.elevator, intermediate.elevator, epsilon);
        Assertions.assertEquals(target.arm, intermediate.arm, epsilon);
        assertThat("Intake forward", intermediate.intake, greaterThanOrEqualTo(0.55));

        current = new Pose(0.3, target.arm, 0.55);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        Assertions.assertEquals(target.elevator, intermediate.elevator, epsilon);
        Assertions.assertEquals(target.arm, intermediate.arm, epsilon);
        assertThat("Intake forward", intermediate.intake, greaterThanOrEqualTo(0.55));

        current = new Pose(target.elevator, target.arm, 0.55);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        Assertions.assertEquals(target.elevator, intermediate.elevator, epsilon);
        Assertions.assertEquals(target.arm, intermediate.arm, epsilon);
        Assertions.assertEquals(target.intake, intermediate.intake, epsilon);
    }

    @Test
    public void ballToHatch() {
        Pose current = SuperStructure.Target.CARGO_BOTTOM;
        Pose target = SuperStructure.Target.HATCH_BOTTOM;

        Pose intermediate = SuperStructure.getIntermediatePose(current, target);

        Assertions.assertEquals(current.elevator, intermediate.elevator, epsilon);
        Assertions.assertEquals(current.arm, intermediate.arm, epsilon);
        assertThat("Intake forward", intermediate.intake, greaterThanOrEqualTo(0.55));

        current = new Pose(current.elevator, current.arm, 0.56);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        assertThat("Elevator raised", intermediate.elevator, greaterThanOrEqualTo(0.3));
        assertThat("Arm raised", intermediate.arm, greaterThanOrEqualTo(0.4));
        assertThat("Intake forward", intermediate.intake, greaterThanOrEqualTo(0.55));

        current = new Pose(0.32, 0.52, 0.56);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        assertThat("Elevator raised", intermediate.elevator, greaterThanOrEqualTo(0.3));
        assertThat("Arm raised", intermediate.arm, greaterThanOrEqualTo(0.4));
        Assertions.assertEquals(target.intake, intermediate.intake, epsilon);

        current = new Pose(0.32, 0.52, target.intake);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        Assertions.assertEquals(target.elevator, intermediate.elevator, epsilon);
        Assertions.assertEquals(target.arm, intermediate.arm, epsilon);
        Assertions.assertEquals(target.intake, intermediate.intake, epsilon);
    }

    @Test
    public void ballBottomToTop() {
        Pose current = SuperStructure.Target.CARGO_BOTTOM;
        Pose target = SuperStructure.Target.CARGO_B_TOP;

        Pose intermediate = SuperStructure.getIntermediatePose(current, target);

        Assertions.assertEquals(current.elevator, intermediate.elevator, epsilon);
        Assertions.assertEquals(current.arm, intermediate.arm, epsilon);
        assertThat("Intake forward", intermediate.intake, greaterThanOrEqualTo(0.55));

        current = new Pose(current.elevator, current.arm, 0.56);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        assertThat("Elevator raised", intermediate.elevator, greaterThanOrEqualTo(0.3));
        assertThat("Arm raised", intermediate.arm, greaterThanOrEqualTo(0.5));
        assertThat("Intake forward", intermediate.intake, greaterThanOrEqualTo(0.55));

        current = new Pose(0.32, 0.52, 0.56);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        Assertions.assertEquals(target.elevator, intermediate.elevator, epsilon);
        Assertions.assertEquals(target.arm, intermediate.arm, epsilon);
        Assertions.assertEquals(target.intake, intermediate.intake, epsilon);
    }

    @Test
    public void ballTopToBottom() {
        Pose current = SuperStructure.Target.CARGO_F_TOP;
        Pose target = SuperStructure.Target.CARGO_BOTTOM;

        Pose intermediate = SuperStructure.getIntermediatePose(current, target);

        assertThat("Intake forward", intermediate.intake, greaterThanOrEqualTo(0.55));
        assertThat("Elevator raised", intermediate.elevator, greaterThanOrEqualTo(0.3));
        assertThat("Arm raised", intermediate.arm, greaterThanOrEqualTo(0.5));

        current = new Pose(0.38, 0.48, 0.56);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        assertThat("Intake forward", intermediate.intake, greaterThanOrEqualTo(0.55));
        Assertions.assertEquals(target.elevator, intermediate.elevator, epsilon);
        Assertions.assertEquals(target.arm, intermediate.arm, epsilon);

        current = new Pose(target.elevator, target.arm, 0.56);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        Assertions.assertEquals(target.elevator, intermediate.elevator, epsilon);
        Assertions.assertEquals(target.arm, intermediate.arm, epsilon);
        Assertions.assertEquals(target.intake, intermediate.intake, epsilon);
    }

    @Test
    public void ballTopToHatchBottom() {
        Pose current = SuperStructure.Target.CARGO_F_TOP;
        Pose target = SuperStructure.Target.HATCH_BOTTOM;

        Pose intermediate = SuperStructure.getIntermediatePose(current, target);

        assertThat("Elevator raised", intermediate.elevator, greaterThanOrEqualTo(0.3));
        Assertions.assertEquals(0.4, intermediate.arm, epsilon);
        Assertions.assertEquals(target.intake, intermediate.intake, epsilon);

        current = new Pose(0.3, 0.5, 0.05);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        Assertions.assertEquals(target.elevator, intermediate.elevator, epsilon);
        Assertions.assertEquals(target.arm, intermediate.arm, epsilon);
        Assertions.assertEquals(target.intake, intermediate.intake, epsilon);
    }

    @Test
    public void ballTopToStart() {
        Pose current = SuperStructure.Target.CARGO_B_TOP;
        Pose target = SuperStructure.Target.STARTING_CONFIG;

        Pose intermediate = SuperStructure.getIntermediatePose(current, target);

        assertThat("Elevator raised", intermediate.elevator, greaterThanOrEqualTo(0.3));
        Assertions.assertEquals(0.4, intermediate.arm, epsilon);
        Assertions.assertEquals(target.intake, intermediate.intake, epsilon);

        current = new Pose(0.3, 0.5, 0.05);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        assertThat("Elevator raised", intermediate.elevator, greaterThanOrEqualTo(0.15));
        Assertions.assertEquals(target.arm, intermediate.arm, epsilon);
        Assertions.assertEquals(target.intake, intermediate.intake, epsilon);

        current = new Pose(0.0, 0.003, 0.03);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        assertThat("Elevator raised", intermediate.elevator, greaterThanOrEqualTo(0.15));
        Assertions.assertEquals(target.arm, intermediate.arm, epsilon);
        Assertions.assertEquals(target.intake, intermediate.intake, epsilon);

        current = new Pose(0.2, 0.003, 0.03);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        assertThat("Elevator raised", intermediate.elevator, greaterThanOrEqualTo(0.15));
        Assertions.assertEquals(target.arm, intermediate.arm, epsilon);
        Assertions.assertEquals(target.intake, intermediate.intake, epsilon);

        current = new Pose(0.2, -0.01, 0.03);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        Assertions.assertEquals(target.elevator, intermediate.elevator, epsilon);
        Assertions.assertEquals(target.arm, intermediate.arm, epsilon);
        Assertions.assertEquals(target.intake, intermediate.intake, epsilon);
    }

    @Test
    public void ballBackToFront() {
        Pose current = SuperStructure.Target.CARGO_M_BACK;
        Pose target = SuperStructure.Target.CARGO_M_FRONT;

        Pose intermediate = SuperStructure.getIntermediatePose(current, target);

        Assertions.assertEquals(target.arm, intermediate.arm, epsilon);
        Assertions.assertEquals(target.arm, intermediate.arm, epsilon);
        assertThat("Intake forward", intermediate.intake, greaterThanOrEqualTo(0.55));

        current = new Pose(0.8, 0.07, 0.55);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        Assertions.assertEquals(target.elevator, intermediate.elevator, epsilon);
        Assertions.assertEquals(target.arm, intermediate.arm, epsilon);
        assertThat("Intake forward", intermediate.intake, greaterThanOrEqualTo(0.55));

        current = new Pose(0.91, 0.07, 0.55);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        Assertions.assertEquals(target.elevator, intermediate.elevator, epsilon);
        Assertions.assertEquals(target.arm, intermediate.arm, epsilon);
        Assertions.assertEquals(target.intake, intermediate.intake, epsilon);
    }

    @Test
    public void ballBottomToStart() {
        Pose current = SuperStructure.Target.CARGO_BOTTOM;
        Pose target = SuperStructure.Target.STARTING_CONFIG;

        Pose intermediate = SuperStructure.getIntermediatePose(current, target);

        Assertions.assertEquals(current.elevator, intermediate.elevator, epsilon);
        Assertions.assertEquals(current.arm, intermediate.arm, epsilon);
        assertThat("Intake forward", intermediate.intake, greaterThanOrEqualTo(0.55));

        current = new Pose(current.elevator, current.arm, 0.56);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        assertThat("Elevator raised", intermediate.elevator, greaterThanOrEqualTo(0.3));
        assertThat("Arm raised", intermediate.arm, greaterThanOrEqualTo(0.4));
        assertThat("Intake forward", intermediate.intake, greaterThanOrEqualTo(0.55));

        current = new Pose(0.32, 0.52, 0.56);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        assertThat("Elevator raised", intermediate.elevator, greaterThanOrEqualTo(0.3));
        assertThat("Arm raised", intermediate.arm, greaterThanOrEqualTo(0.4));
        Assertions.assertEquals(target.intake, intermediate.intake, epsilon);

        current = new Pose(0.32, 0.52, 0.05);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        assertThat("Elevator raised", intermediate.elevator, greaterThanOrEqualTo(0.15));
        Assertions.assertEquals(target.arm, intermediate.arm, epsilon);
        Assertions.assertEquals(target.intake, intermediate.intake, epsilon);

        current = new Pose(0.0, 0.003, 0.03);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        assertThat("Elevator raised", intermediate.elevator, greaterThanOrEqualTo(0.15));
        Assertions.assertEquals(target.arm, intermediate.arm, epsilon);
        Assertions.assertEquals(target.intake, intermediate.intake, epsilon);

        current = new Pose(0.2, 0.003, 0.03);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        assertThat("Elevator raised", intermediate.elevator, greaterThanOrEqualTo(0.15));
        Assertions.assertEquals(target.arm, intermediate.arm, epsilon);
        Assertions.assertEquals(target.intake, intermediate.intake, epsilon);

        current = new Pose(0.2, -0.01, 0.03);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        Assertions.assertEquals(target.elevator, intermediate.elevator, epsilon);
        Assertions.assertEquals(target.arm, intermediate.arm, epsilon);
        Assertions.assertEquals(target.intake, intermediate.intake, epsilon);
    }

    @Test
    public void hatchToClimb() {
        Pose current = SuperStructure.Target.HATCH_BOTTOM;
        Pose target = SuperStructure.Target.PRE_CLIMB;

        Pose intermediate = SuperStructure.getIntermediatePose(current, target);

        assertThat("Elevator raised", intermediate.elevator, greaterThanOrEqualTo(0.3));
        assertThat("Arm raised", intermediate.arm, greaterThanOrEqualTo(0.3));
        Assertions.assertEquals(current.intake, intermediate.intake, epsilon);

        current = new Pose(0.975, current.arm, current.intake);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        assertThat("Elevator raised", intermediate.elevator, greaterThanOrEqualTo(0.3));
        assertThat("Arm raised", intermediate.arm, greaterThanOrEqualTo(0.3));
        Assertions.assertEquals(current.intake, intermediate.intake, epsilon);

        current = new Pose(0.975, 0.45, 0.55);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        Assertions.assertEquals(target.elevator, intermediate.elevator, epsilon);
        Assertions.assertEquals(target.arm, intermediate.arm, epsilon);
        assertThat("Intake forward", intermediate.intake, greaterThanOrEqualTo(0.55));

        current = new Pose(0.3, 0.45, 0.55);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        Assertions.assertEquals(target.elevator, intermediate.elevator, epsilon);
        Assertions.assertEquals(target.arm, intermediate.arm, epsilon);
        assertThat("Intake forward", intermediate.intake, greaterThanOrEqualTo(0.55));

        current = new Pose(target.elevator, target.arm, 0.55);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        Assertions.assertEquals(target.elevator, intermediate.elevator, epsilon);
        Assertions.assertEquals(target.arm, intermediate.arm, epsilon);
        Assertions.assertEquals(target.intake, intermediate.intake, epsilon);
    }

    @Test
    public void climbToHatchTop() {
        Pose current = SuperStructure.Target.PRE_CLIMB;
        Pose target = SuperStructure.Target.HATCH_TOP;

        Pose intermediate = SuperStructure.getIntermediatePose(current, target);

        Assertions.assertEquals(current.elevator, intermediate.elevator, epsilon);
        Assertions.assertEquals(current.arm, intermediate.arm, epsilon);
        assertThat("Intake forward", intermediate.intake, greaterThanOrEqualTo(0.55));

        current = new Pose(current.elevator, current.arm, 0.55);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        assertThat("Elevator raised", intermediate.elevator, greaterThanOrEqualTo(0.3));
        assertThat("Arm raised", intermediate.arm, greaterThanOrEqualTo(0.4));
        assertThat("Intake forward", intermediate.intake, greaterThanOrEqualTo(0.55));

        current = new Pose(0.32, 0.52, 0.54);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        Assertions.assertEquals(target.elevator, intermediate.elevator, epsilon);
        Assertions.assertEquals(target.arm, intermediate.arm, epsilon);
        Assertions.assertEquals(target.intake, intermediate.intake, epsilon);
    }

    @Test
    public void climbToCargoBottom() {
        Pose current = SuperStructure.Target.PRE_CLIMB;
        Pose target = SuperStructure.Target.CARGO_BOTTOM;

        Pose intermediate = SuperStructure.getIntermediatePose(current, target);

        Assertions.assertEquals(target.elevator, intermediate.elevator, epsilon);
        Assertions.assertEquals(target.arm, intermediate.arm, epsilon);
        Assertions.assertEquals(target.intake, intermediate.intake, epsilon);

        current = new Pose(current.elevator, current.arm, 0.55);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        Assertions.assertEquals(target.elevator, intermediate.elevator, epsilon);
        Assertions.assertEquals(target.arm, intermediate.arm, epsilon);
        Assertions.assertEquals(target.intake, intermediate.intake, epsilon);
    }

    @Test
    public void climbToHatchBack() {
        Pose current = SuperStructure.Target.PRE_CLIMB;
        Pose target = SuperStructure.Target.HATCH_M_BACK;

        Pose intermediate = SuperStructure.getIntermediatePose(current, target);

        Assertions.assertEquals(current.elevator, intermediate.elevator, epsilon);
        Assertions.assertEquals(current.arm, intermediate.arm, epsilon);
        assertThat("Intake forward", intermediate.intake, greaterThanOrEqualTo(0.55));

        current = new Pose(current.elevator, current.arm, current.intake + 0.02);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        Assertions.assertEquals(current.elevator, intermediate.elevator, epsilon);
        Assertions.assertEquals(current.arm, intermediate.arm, epsilon);
        assertThat("Intake forward", intermediate.intake, greaterThanOrEqualTo(0.55));

        current = new Pose(current.elevator, current.arm, 0.56);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        assertThat("Elevator raised", intermediate.elevator, greaterThanOrEqualTo(0.3));
        assertThat("Arm raised", intermediate.arm, greaterThanOrEqualTo(0.4));
        assertThat("Intake forward", intermediate.intake, greaterThanOrEqualTo(0.55));

        current = new Pose(target.elevator, 0.61, 0.56);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        Assertions.assertEquals(target.elevator, intermediate.elevator, epsilon);
        Assertions.assertEquals(target.arm, intermediate.arm, epsilon);
        Assertions.assertEquals(target.intake, intermediate.intake, epsilon);
    }
}
