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

    public interface StatusCheck {
        public boolean get();
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

    private synchronized void setTarget(double targetOffset) {
        this.targetOffset = targetOffset;
    }

    public synchronized double getOffset() {
        return targetOffset;
    }

    public void clear() {
        if(port == null) {
            initPort();
        }

        if(port != null) {
            port.readString();
        }
    }

    private Thread createThread() {
        return new Thread(() -> {
            while (!initialized && !Thread.interrupted()) {
                Timer.delay(0.2);
                initPort();
            }

            while (!Thread.interrupted() && initialized) {
                try {
                    parseInput(remainingInput + port.readString());
                } catch (Exception e) {
                    System.out.println(e.toString());
                    initPort();
                    initialized = true;
                }
                Timer.delay(0.034);
            }
        });
    }

    private void parseInput(String input) {
        int offsetIndex = input.lastIndexOf("OFF");
        int endIndex = input.lastIndexOf("END");

        if (endIndex > offsetIndex && offsetIndex > 0) {
            setTarget(Double.valueOf(input.substring(offsetIndex + 4, endIndex - 1)));
            remainingInput = input.substring(endIndex + 3);
        } else {
            remainingInput = input;
        }
    }

    private void initPort() {
        if (port != null) {
            try {
                port.close();
            } catch (Exception e) {
            }
        }

        try {
            port = new SerialPort(115200, SerialPort.Port.kUSB1);
            initialized = true;
        } catch (Exception e) {
            initialized = false;
        }
    }
}