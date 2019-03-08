package frc.robot.utils;

public class Bounds {
    private double max, min;

    public Bounds(double min, double max) {
        this.max = Math.max(min, max);
        this.min = Math.min(min, max);
    }

    public static Bounds noBounds() {
        return new Bounds(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
    }

    public double max() {
        return max;
    }

    public double min() {
        return min;
    }

    public double size() {
        return max - min;
    }

    public double clamp(double value) {
        if (value < min) {
            return min;
        } else if (value > max) {
            return max;
        }
        return value;
    }

    public boolean overlaps(Bounds other) {
        return min <= other.max() && other.min() <= max;
    }

    public boolean contains(double value) {
        return min <= value && value <= max;
    }
}