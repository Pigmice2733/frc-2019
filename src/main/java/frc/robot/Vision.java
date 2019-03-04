package frc.robot;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;

public class Vision {
    private SerialPort port;
    private Thread thread;
    private StatusCheck enabledStatus;
    private boolean initialized = false;

    private String remainingInput = "";
    private volatile double targetOffset = 0.0;
    private volatile double targetDistance = 0.0;

    public interface StatusCheck {
        public boolean get();
    }

    public class Target {
        public final double distance;
        public final double offset;

        public Target(double distance, double offset) {
            this.distance = distance;
            this.offset = offset;
        }
    }

    public Vision(StatusCheck enabled) {
        enabledStatus = enabled;

        thread = createThread();
    }

    public void start() {
        if (!thread.isAlive()) {
            thread.start();
        }
    }

    public void stop() {
        thread.interrupt();
        thread = createThread();
    }

    private synchronized void setTarget(double targetDistance, double targetOffset) {
        this.targetDistance = targetDistance;
        this.targetOffset = targetOffset;
    }

    public synchronized Target getTarget() {
        return new Target(targetDistance, targetOffset);
    }

    private Thread createThread() {
        return new Thread(() -> {
            while (!initialized && !Thread.interrupted() && enabledStatus.get()) {
                Timer.delay(0.2);
                initPort();
                System.out.println("disconnected");
            }

            while (!Thread.interrupted() && initialized && enabledStatus.get()) {
                try {
                    parseInput(remainingInput + port.readString());
                } catch (Exception e) {
                    System.out.println(e.toString());
                    initPort();
                }
                Timer.delay(0.034);
            }

            System.out.println("disconnected");
        });
    }

    private void parseInput(String input) {
        int offsetIndex = input.lastIndexOf("OFF");
        int distanceIndex = input.lastIndexOf("DIST");
        int endIndex = input.lastIndexOf("END");

        if (endIndex > distanceIndex && distanceIndex > offsetIndex && offsetIndex > 0) {
            setTarget(Double.valueOf(input.substring(distanceIndex + 5, endIndex - 1)),
                    Double.valueOf(input.substring(offsetIndex + 4, distanceIndex - 1)));
            remainingInput = input.substring(endIndex + 3);
        } else {
            remainingInput = input;
        }
    }

    private void initPort() {
        try {
            port = new SerialPort(9600, SerialPort.Port.kUSB1);
            initialized = true;
        } catch (Exception e) {
            initialized = false;
        }
    }
}