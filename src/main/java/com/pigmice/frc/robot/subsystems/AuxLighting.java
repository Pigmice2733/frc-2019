package com.pigmice.frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;

public class AuxLighting {
    private Spark blinkin;
    private Color currentColor;
    private Color baseColor;

    public enum Color {
        DEFAULT(0.91), RED(0.61), BLUE(0.87), ORANGE(0.65);

        private final Double value;

        Color(double value) {
            this.value = value;
        }
    }

    public AuxLighting(int port) {
        blinkin = new Spark(port);

        setBaseColor(Color.DEFAULT);
        setColor(Color.DEFAULT);
    }

    public void setColor(Color color) {
        if (color != currentColor) {
            currentColor = color;
            blinkin.set(currentColor.value);
        }
    }

    public void setBaseColor(Color baseColor) {
        this.baseColor = baseColor;
    }

    public void resetToBase() {
        setColor(this.baseColor);
    }
}
