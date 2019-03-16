package frc.robot.superstructure;

import org.junit.Assert;
import org.junit.Test;
import static org.hamcrest.Matchers.*;

public class SuperStructureTest {
    private static final double epsilon = 1e-6;

    @Test
    public void startToHatch() {
        Pose current = SuperStructure.Target.STARTING_CONFIG;
        Pose target = SuperStructure.Target.HATCH_BOTTOM;

        Pose intermediate = SuperStructure.getIntermediatePose(current, target);

        Assert.assertThat("Elevator raised", intermediate.elevator, greaterThanOrEqualTo(0.2));
        Assert.assertEquals(current.arm, intermediate.arm, epsilon);
        Assert.assertTrue(intermediate.intake < 0.115 && intermediate.intake >= 0.0);

        current = new Pose(0.16, current.arm, 0.05);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        Assert.assertThat("Elevator raised", intermediate.elevator, greaterThanOrEqualTo(0.2));
        Assert.assertEquals(target.arm, intermediate.arm, epsilon);
        Assert.assertTrue(intermediate.intake < 0.115 && intermediate.intake >= 0.0);

        current = new Pose(0.16, 0.01, 0.05);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        Assert.assertEquals(target.elevator, intermediate.elevator, epsilon);
        Assert.assertEquals(target.arm, intermediate.arm, epsilon);
        Assert.assertTrue(intermediate.intake < 0.115 && intermediate.intake >= 0.0);
    }

    @Test
    public void hatchTopToStart() {
        Pose current = SuperStructure.Target.HATCH_M_BACK;
        Pose target = SuperStructure.Target.STARTING_CONFIG;

        Pose intermediate = SuperStructure.getIntermediatePose(current, target);

        Assert.assertThat("Elevator raised", intermediate.elevator, greaterThanOrEqualTo(0.2));
        Assert.assertEquals(target.arm, intermediate.arm, epsilon);
        Assert.assertTrue(intermediate.intake < 0.115 && intermediate.intake >= 0.0);

        current = new Pose(0.1, 0.001, 0.05);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        Assert.assertThat("Elevator raised", intermediate.elevator, greaterThanOrEqualTo(0.2));
        Assert.assertTrue(intermediate.arm > 0.001);
        Assert.assertTrue(intermediate.intake < 0.115 && intermediate.intake >= 0.0);

        current = new Pose(0.2, 0.001, 0.05);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        Assert.assertThat("Elevator raised", intermediate.elevator, greaterThanOrEqualTo(0.2));
        Assert.assertEquals(target.arm, intermediate.arm, epsilon);
        Assert.assertTrue(intermediate.intake < 0.115 && intermediate.intake >= 0.0);

        current = new Pose(0.16, -0.001, 0.05);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        Assert.assertEquals(target.elevator, intermediate.elevator, epsilon);
        Assert.assertEquals(target.arm, intermediate.arm, epsilon);
        Assert.assertTrue(intermediate.intake < 0.115 && intermediate.intake >= 0.0);
    }

    @Test
    public void hatchBottomToTop() {
        Pose current = SuperStructure.Target.HATCH_BOTTOM;
        Pose target = SuperStructure.Target.HATCH_TOP;

        Pose intermediate = SuperStructure.getIntermediatePose(current, target);

        Assert.assertEquals(target.elevator, intermediate.elevator, epsilon);
        Assert.assertEquals(target.arm, intermediate.arm, epsilon);
        Assert.assertEquals(target.intake, intermediate.intake, epsilon);
    }

    @Test
    public void hatchToBall() {
        Pose current = SuperStructure.Target.HATCH_BOTTOM;
        Pose target = SuperStructure.Target.CARGO_BOTTOM;

        Pose intermediate = SuperStructure.getIntermediatePose(current, target);

        Assert.assertThat("Elevator raised", intermediate.elevator, greaterThan(0.875));
        Assert.assertThat("Arm lowered", intermediate.arm, lessThanOrEqualTo(0.1));
        Assert.assertEquals(current.intake, intermediate.intake, epsilon);

        current = new Pose(0.975, current.arm, current.intake);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        Assert.assertThat("Elevator raised", intermediate.elevator, greaterThan(0.875));
        Assert.assertThat("Arm lowered", intermediate.arm, lessThanOrEqualTo(0.1));
        Assert.assertThat("Intake forward", intermediate.intake, greaterThanOrEqualTo(0.55));

        current = new Pose(0.975, target.arm, 0.55);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        Assert.assertEquals(target.elevator, intermediate.elevator, epsilon);
        Assert.assertEquals(target.arm, intermediate.arm, epsilon);
        Assert.assertThat("Intake forward", intermediate.intake, greaterThanOrEqualTo(0.55));

        current = new Pose(0.4, target.arm, 0.55);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        Assert.assertEquals(target.elevator, intermediate.elevator, epsilon);
        Assert.assertEquals(target.arm, intermediate.arm, epsilon);
        Assert.assertThat("Intake forward", intermediate.intake, greaterThanOrEqualTo(0.55));

        current = new Pose(target.elevator, target.arm, 0.55);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        Assert.assertEquals(target.elevator, intermediate.elevator, epsilon);
        Assert.assertEquals(target.arm, intermediate.arm, epsilon);
        Assert.assertEquals(target.intake, intermediate.intake, epsilon);
    }

    @Test
    public void ballBottomToTop() {
        Pose current = SuperStructure.Target.CARGO_BOTTOM;
        Pose target = SuperStructure.Target.CARGO_TOP;

        Pose intermediate = SuperStructure.getIntermediatePose(current, target);

        Assert.assertEquals(current.elevator, intermediate.elevator, epsilon);
        Assert.assertEquals(current.arm, intermediate.arm, epsilon);
        Assert.assertThat("Intake forward", intermediate.intake, greaterThanOrEqualTo(0.55));

        current = new Pose(current.elevator, current.arm, 0.56);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        Assert.assertEquals(target.elevator, intermediate.elevator, epsilon);
        Assert.assertEquals(target.arm, intermediate.arm, epsilon);
        Assert.assertThat("Intake forward", intermediate.intake, greaterThanOrEqualTo(0.55));

        current = new Pose(0.42, 0.52, 0.56);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        Assert.assertEquals(target.elevator, intermediate.elevator, epsilon);
        Assert.assertEquals(target.arm, intermediate.arm, epsilon);
        Assert.assertEquals(target.intake, intermediate.intake, epsilon);
    }

    @Test
    public void ballTopToBottom() {
        Pose current = SuperStructure.Target.CARGO_TOP;
        Pose target = SuperStructure.Target.CARGO_BOTTOM;

        Pose intermediate = SuperStructure.getIntermediatePose(current, target);

        Assert.assertThat("Intake forward", intermediate.intake, greaterThanOrEqualTo(0.55));
        Assert.assertThat("Elevator raised", intermediate.elevator, greaterThanOrEqualTo(0.3));
        Assert.assertEquals(0.5, intermediate.arm, epsilon);

        current = new Pose(0.38, 0.48, 0.56);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        Assert.assertThat("Intake forward", intermediate.intake, greaterThanOrEqualTo(0.55));
        Assert.assertEquals(target.elevator, intermediate.elevator, epsilon);
        Assert.assertEquals(target.arm, intermediate.arm, epsilon);

        current = new Pose(target.elevator, target.arm, 0.56);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        Assert.assertEquals(target.elevator, intermediate.elevator, epsilon);
        Assert.assertEquals(target.arm, intermediate.arm, epsilon);
        Assert.assertEquals(target.intake, intermediate.intake, epsilon);
    }

    @Test
    public void ballTopToHatchBottom() {
        Pose current = SuperStructure.Target.CARGO_TOP;
        Pose target = SuperStructure.Target.HATCH_BOTTOM;

        Pose intermediate = SuperStructure.getIntermediatePose(current, target);

        Assert.assertThat("Elevator raised", intermediate.elevator, greaterThanOrEqualTo(0.3));
        Assert.assertEquals(0.5, intermediate.arm, epsilon);
        Assert.assertEquals(target.intake, intermediate.intake, epsilon);

        current = new Pose(0.4, 0.5, 0.05);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        Assert.assertEquals(target.elevator, intermediate.elevator, epsilon);
        Assert.assertEquals(target.arm, intermediate.arm, epsilon);
        Assert.assertEquals(target.intake, intermediate.intake, epsilon);
    }

    @Test
    public void ballTopToStart() {
        Pose current = SuperStructure.Target.CARGO_TOP;
        Pose target = SuperStructure.Target.STARTING_CONFIG;

        Pose intermediate = SuperStructure.getIntermediatePose(current, target);

        Assert.assertThat("Elevator raised", intermediate.elevator, greaterThanOrEqualTo(0.3));
        Assert.assertEquals(0.5, intermediate.arm, epsilon);
        Assert.assertEquals(target.intake, intermediate.intake, epsilon);

        current = new Pose(0.4, 0.5, 0.05);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        Assert.assertThat("Elevator raised", intermediate.elevator, greaterThanOrEqualTo(0.2));
        Assert.assertEquals(target.arm, intermediate.arm, epsilon);
        Assert.assertEquals(target.intake, intermediate.intake, epsilon);

        current = new Pose(0.0, 0.003, 0.03);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        Assert.assertThat("Elevator raised", intermediate.elevator, greaterThanOrEqualTo(0.2));
        Assert.assertEquals(0.005, intermediate.arm, epsilon);
        Assert.assertEquals(target.intake, intermediate.intake, epsilon);

        current = new Pose(0.2, 0.003, 0.03);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        Assert.assertEquals(0.2, intermediate.elevator, epsilon);
        Assert.assertEquals(target.arm, intermediate.arm, epsilon);
        Assert.assertEquals(target.intake, intermediate.intake, epsilon);

        current = new Pose(0.2, -0.01, 0.03);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        Assert.assertEquals(target.elevator, intermediate.elevator, epsilon);
        Assert.assertEquals(target.arm, intermediate.arm, epsilon);
        Assert.assertEquals(target.intake, intermediate.intake, epsilon);
    }

    @Test
    public void ballBottomToStart() {
        Pose current = SuperStructure.Target.CARGO_BOTTOM;
        Pose target = SuperStructure.Target.STARTING_CONFIG;

        Pose intermediate = SuperStructure.getIntermediatePose(current, target);

        Assert.assertEquals(current.elevator, intermediate.elevator, epsilon);
        Assert.assertEquals(current.arm, intermediate.arm, epsilon);
        Assert.assertThat("Intake forward", intermediate.intake, greaterThanOrEqualTo(0.55));

        current = new Pose(current.elevator, current.arm, 0.56);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        Assert.assertThat("Elevator raised", intermediate.elevator, greaterThanOrEqualTo(0.4));
        Assert.assertThat("Arm raised", intermediate.arm, greaterThanOrEqualTo(0.5));
        Assert.assertThat("Intake forward", intermediate.intake, greaterThanOrEqualTo(0.55));

        current = new Pose(0.42, 0.52, 0.56);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        Assert.assertThat("Elevator raised", intermediate.elevator, greaterThanOrEqualTo(0.4));
        Assert.assertThat("Arm raised", intermediate.arm, greaterThanOrEqualTo(0.5));
        Assert.assertEquals(target.intake, intermediate.intake, epsilon);

        current = new Pose(0.42, 0.52, 0.05);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        Assert.assertThat("Elevator raised", intermediate.elevator, greaterThanOrEqualTo(0.2));
        Assert.assertEquals(target.arm, intermediate.arm, epsilon);
        Assert.assertEquals(target.intake, intermediate.intake, epsilon);

        current = new Pose(0.0, 0.003, 0.03);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        Assert.assertThat("Elevator raised", intermediate.elevator, greaterThanOrEqualTo(0.2));
        Assert.assertEquals(0.005, intermediate.arm, epsilon);
        Assert.assertEquals(target.intake, intermediate.intake, epsilon);

        current = new Pose(0.2, 0.003, 0.03);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        Assert.assertThat("Elevator raised", intermediate.elevator, greaterThanOrEqualTo(0.2));
        Assert.assertEquals(target.arm, intermediate.arm, epsilon);
        Assert.assertEquals(target.intake, intermediate.intake, epsilon);

        current = new Pose(0.2, -0.01, 0.03);
        intermediate = SuperStructure.getIntermediatePose(current, target);

        Assert.assertEquals(target.elevator, intermediate.elevator, epsilon);
        Assert.assertEquals(target.arm, intermediate.arm, epsilon);
        Assert.assertEquals(target.intake, intermediate.intake, epsilon);
    }
}
