package frc.robot.utils;

/**
 * A ring is a circular FIFO structure with a fixed size. Writing to the ring
 * will overwrite the oldest data when the ring is full. Reading from a ring
 * gives the oldest data point that hasn't been overwritten, and replaces it
 * with 0.0.
 */
public class Ring {
    private final double[] storage;

    private int writePos = 0;
    private int readPos = 0;

    private final int size;

    private double total = 0.0;
    private double average = 0.0;

    /**
     *
     * @param size The maximum number of elements the ring can store, > 0
     * @throws IllegalArgumentException if ring size is < 1
     */
    public Ring(int size) throws IllegalArgumentException {
        if (size < 1) {
            throw new IllegalArgumentException("Size of ring must be positive");
        }

        this.size = size;
        storage = new double[size];
    }

    /**
     * Add an element to the next empty space in the ring, or overwrite the oldest
     * data if the ring is full.
     *
     * @param value The number to add to the ring.
     */
    public void put(double value) {
        if (readPos == writePos % size) {
            readPos = (readPos + 1) % size;
        }

        total -= storage[writePos];
        storage[writePos] = value;

        writePos = (writePos + 1) % size;

        total += value;
        average = total / size;
    }

    /**
     * Returns the oldest data point from the ring and replaces it with 0.0. If the
     * ring
     * is empty, returns 0.0.
     *
     * @return The oldest data point from the ring.
     */
    public double pop() {
        if (writePos == readPos % size) {
            writePos = (writePos + 1) % size;
        }

        double value = storage[readPos];
        storage[readPos] = 0.0;

        readPos = (readPos + 1) % size;

        total -= value;
        average = total / size;

        return value;
    }

    /**
     * Returns the current average of all the elements in the ring.
     *
     * @return The current average.
     */
    public double average() {
        return average;
    }
}
