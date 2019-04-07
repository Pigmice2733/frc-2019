package frc.robot;

import java.util.TimerTask;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;

public class Vision {
    private static SerialPort port;

    private static final ScheduledExecutorService scheduler = Executors.newScheduledThreadPool(1, new ThreadFactory() {
        public Thread newThread(Runnable r) {
            Thread t = Executors.defaultThreadFactory().newThread(r);
            t.setDaemon(true);
            return t;
        }
    });

    private static volatile ScheduledFuture<?> self;
    private static boolean initialized = false;
    private static boolean enabled;

    private static boolean connected = false;
    // Time of last message (seconds)
    private static double lastMessage = 0.0;

    // Maximum time between messages that still counts as connected (seconds)
    private static final double disconnectTime = 0.25;

    // Compensation factor for camera being off center, in fraction of distance
    // between vision targets
    private static final double cameraOffsetCompensation = 0.32;

    // Roll-over input from camera since messages don't always line up with
    private static String remainingInput = "";
    private static volatile double targetOffset = 0.0;
    private static volatile boolean targetVisible = false;

    public synchronized static void start() {
        if (!enabled) {
            enabled = true;
            self = scheduler.scheduleAtFixedRate(Vision::update, 1000, 60, TimeUnit.MILLISECONDS);
        }
    }

    public synchronized static void stop() {
        enabled = false;
        self.cancel(true);
        self = null;
        port = null;
        connected = false;
        targetOffset = 0.0;
        targetVisible = false;
    }

    public synchronized static double getOffset() {
        return targetOffset;
    }

    public synchronized static boolean targetVisible() {
        return connected && targetVisible;
    }

    private synchronized static void setTarget(double targetOffset, boolean targetVisible) {
        Vision.targetOffset = targetOffset;
        Vision.targetVisible = targetVisible;
    }

    private synchronized static void setConnected(boolean connected) {
        Vision.connected = connected;
    }

    private static void update() {
        double currentTime = Timer.getFPGATimestamp();
        if (!initialized) {
            initPort();
        } else {
            try {
                if (port.getBytesReceived() > 0) {
                    lastMessage = currentTime;
                    parseInput(remainingInput + port.readString());
                }
            } catch (Exception e) {
                System.out.println(e.toString());
                initPort();
            }
        }

        if (currentTime - lastMessage > disconnectTime) {
            setConnected(false);
        }
    }

    private static void parseInput(String input) {
        int startIndex = input.lastIndexOf("START");
        int separatorIndex = input.lastIndexOf(",");
        int endIndex = input.lastIndexOf("END");

        if (endIndex > startIndex && startIndex > -1) {
            String centerString = input.substring(startIndex + 6, separatorIndex);
            if (centerString.length() == 4 && centerString.toLowerCase().equals("None")) {
                setTarget(0.0, false);
            } else {
                double center = Double.valueOf(centerString);
                double width = Double.valueOf(input.substring(separatorIndex + 2, endIndex));
                setTarget(center + width * cameraOffsetCompensation, true);
            }
            remainingInput = input.substring(endIndex + 3);
        } else {
            remainingInput = input;
        }
    }

    private static void initPort() {
        if (port == null) {
            try {
                port = new SerialPort(115200, SerialPort.Port.kUSB1);
                initialized = true;
            } catch (Exception e) {
                port = null;
                initialized = false;
            }
        } else if (!initialized) {
            try {
                port.reset();
                initialized = true;
            } catch (Exception e) {
                initialized = false;
            }
        }
    }
}
