package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NTStreamer<E> {
    private NetworkTableEntry entry;

    private E value;

    public NTStreamer(String table, String key) {
        NetworkTableInstance root = NetworkTableInstance.getDefault();
        entry = root.getTable(table).getEntry(key);
    }

    public void send(E value) {
        if (value instanceof Boolean) {
            if (this.value != value) {
                entry.setBoolean((boolean) value);
            }
        } else if (value instanceof Double) {
            if (Math.abs((double) this.value - (double) value) < 1e-6) {
                entry.setDouble((double) value);
            }
        } else if (value instanceof String) {
            if (this.value != value) {
                entry.setString(String.valueOf(value));
            }
        }
    }
}