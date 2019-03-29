package frc.robot.utils;

public class Ring {
    private final double[] storage;

    private int writePos = 0;
    private int readPos = 0;

    private final int size;

    public Ring(int size) {
        if (size < 1) {
            throw new IllegalArgumentException("Size of ring must be positive");
        }

        this.size = size;
        storage = new double[size];
    }

    public void put(double value) {
        storage[writePos] = value;

        writePos = (writePos + 1) % size;
    }

    public double get() {
        double value = storage[readPos];

        readPos = (readPos + 1) % size;
        return value;
    }
}