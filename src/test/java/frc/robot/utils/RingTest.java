package frc.robot.utils;

import org.junit.Test;
import org.junit.Assert;

public class RingTest {
    private static final double epsilon = 1e-6;

    @Test
    public void noLoop() {
        Ring ring = new Ring(5);

        ring.put(0.0);
        ring.put(1.0);
        ring.put(2.0);
        ring.put(3.0);
        ring.put(4.0);

        Assert.assertEquals(0.0, ring.get(), epsilon);
        Assert.assertEquals(1.0, ring.get(), epsilon);
        Assert.assertEquals(2.0, ring.get(), epsilon);
        Assert.assertEquals(3.0, ring.get(), epsilon);
        Assert.assertEquals(4.0, ring.get(), epsilon);
    }

    @Test
    public void loop() {
        Ring ring = new Ring(4);

        ring.put(0.0);
        ring.put(1.0);
        ring.put(2.0);
        ring.put(3.0);
        ring.put(4.0);

        Assert.assertEquals(4.0, ring.get(), epsilon);
        Assert.assertEquals(1.0, ring.get(), epsilon);
        Assert.assertEquals(2.0, ring.get(), epsilon);
        Assert.assertEquals(3.0, ring.get(), epsilon);
        Assert.assertEquals(4.0, ring.get(), epsilon);
        Assert.assertEquals(1.0, ring.get(), epsilon);
        Assert.assertEquals(2.0, ring.get(), epsilon);
    }

    @Test(expected = IllegalArgumentException.class)
    public void invalidSize() {
        Ring ring = new Ring(0);
    }
}