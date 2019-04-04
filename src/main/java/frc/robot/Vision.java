package frc.robot;

import java.util.TimerTask;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj.SerialPort;

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

    private static String remainingInput = "";
    private static volatile double targetOffset = 0.0;

    public static void start() {
        if (!enabled) {
            enabled = true;
            self = scheduler.scheduleAtFixedRate(Vision::update, 1000, 5, TimeUnit.MILLISECONDS);
        }
    }

    public static void stop() {
        enabled = false;
    }

    public synchronized static double getOffset() {
        return targetOffset;
    }

    public static void clearUsbBuffer() {
        if (port == null) {
            initPort();
        }

        if (port != null) {
            port.readString();
        }
    }

    private synchronized static void setTarget(double targetOffset) {
        Vision.targetOffset = targetOffset;
    }

    private static void update() {
        if (!enabled) {
            self.cancel(false);
            self = null;
            port = null;
            return;
        }

        if (!initialized) {
            initPort();
        } else {
            try {
                parseInput(remainingInput + port.readString());
            } catch (Exception e) {
                System.out.println(e.toString());
                initPort();
                initialized = true;
            }

            port.writeString("set-mask on\n");
        }
    }

    private static void parseInput(String input) {
        int offsetIndex = input.lastIndexOf("OFF");
        int endIndex = input.lastIndexOf("END");

        if (endIndex > offsetIndex && offsetIndex > 0) {
            setTarget(Double.valueOf(input.substring(offsetIndex + 4, endIndex - 1)));
            remainingInput = input.substring(endIndex + 3);
        } else {
            remainingInput = input;
        }
    }

    private static void initPort() {
        if (port != null) {
            try {
                port.reset();
            } catch (Exception e) {
            }
        } else {
            try {
                port = new SerialPort(115200, SerialPort.Port.kUSB1);
                initialized = true;
            } catch (Exception e) {
                initialized = false;
            }
        }
    }
}