package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Odometry {
    public static class OdometryStreamer {
        private NetworkTableEntry xEntry;
        private NetworkTableEntry yEntry;
        private NetworkTableEntry angleEntry;

        private double x, y, angle;

        public OdometryStreamer(String table) {
            NetworkTableInstance root = NetworkTableInstance.getDefault();
            NetworkTable baseTable = root.getTable(table);
            xEntry = baseTable.getEntry("x");
            yEntry = baseTable.getEntry("y");
            angleEntry = baseTable.getEntry("angle");
        }

        public void send(double x, double y, double angle) {
            if (Math.abs(this.x - x) < 1e-6) {
                xEntry.setDouble(x);
            }

            if (Math.abs(this.y - y) < 1e-6) {
                yEntry.setDouble(y);
            }

            if (Math.abs(this.angle - angle) < 1e-6) {
                angleEntry.setDouble(angle);
            }
        }
    }

    private double x, y;

    private double lastLeft, lastRight;
    private double lastAngle;

    private OdometryStreamer streamer;

    public Point getPosition() {
        return new Point(x, y);
    }

    public Odometry(double x, double y, double angle) {
        lastLeft = 0;
        lastRight = 0;
        lastAngle = angle;
        this.x = x;
        this.y = y;
    }

    public void update(double leftPosition, double rightPosition, double angle) {
        double deltaLeft = leftPosition - lastLeft;
        double deltaRight = rightPosition - lastRight;
        double deltaAngle = (angle - lastAngle) * (Math.PI / 180.0);

        double distance = (deltaLeft + deltaRight) / 2.0;
        if (Math.abs(deltaAngle) > Math.PI) {
            deltaAngle = Math.copySign(2.0 * Math.PI - Math.abs(deltaAngle), -deltaAngle);
        }

        if (Math.abs(deltaAngle) < 1e-6) {
            double deltaX = distance * Math.cos(angle);
            double deltaY = distance * Math.sin(angle);
            x += deltaX;
            y += deltaY;

            lastAngle = angle;

            streamer.send(x, y, angle);
            return;
        }

        double radius = distance / Math.abs(deltaAngle);
        double chordAngle = lastAngle + deltaAngle / 2.0;
        double chordLength = 2.0 * radius * Math.sin(Math.abs(deltaAngle) / 2.0);

        double deltaX = chordLength * Math.cos(chordAngle);
        double deltaY = chordLength * Math.sin(chordAngle);

        x += deltaX;
        y += deltaY;

        lastAngle = angle;

        streamer.send(x, y, angle);
        return;
    }
}