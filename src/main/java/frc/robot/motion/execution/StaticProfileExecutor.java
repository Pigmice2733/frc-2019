package frc.robot.motion.execution;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.motion.Setpoint;
import frc.robot.motion.StaticProfile;

public class StaticProfileExecutor {
    public interface Output {
        void set(Setpoint sp);
    }

    public interface Input {
        double get();
    }

    private StaticProfile profile;
    private Output output;
    private Input input;
    private double startTime;
    private double allowableError;

    public StaticProfileExecutor(StaticProfile profile, Output output, Input input, double allowableError) {
        this.profile = profile;
        this.output = output;
        this.input = input;
        this.allowableError = allowableError;
    }

    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    // Returns true if error is within the allowable error, false otherwise
    public boolean update() {
        double time = Timer.getFPGATimestamp() - startTime;
        Setpoint sp = profile.getSetpoint(time);
        output.set(sp);

        // return time >= profile.getDuration();
        double error = Math.abs(profile.getPosition(profile.getDuration()) - input.get());
        return error <= allowableError;
    }
}