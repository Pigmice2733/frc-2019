package frc.robot.utils;

public class Ring {
    private final double[] storage;

    private int writePos = 0;
    private int readPos = 0;

    private final int size;

    private double total = 0.0;
    private double average = 0.0;

    public Ring(int size) {
        if (size < 1) {
            throw new IllegalArgumentException("Size of ring must be positive");
        }

        this.size = size;
        storage = new double[size];
    }

    public void put(double value) {
        total -= storage[writePos];

        storage[writePos] = value;

        writePos = (writePos + 1) % size;

        total += value;

        average = total / size;
    }

    public double get() {
        double value = storage[readPos];
        storage[readPos] = 0.0;

        total -= value;
        average = total / size;

        readPos = (readPos + 1) % size;
        return value;
    }

    public double average() {
        return average;
    }
}