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

    private double finalTarget;

    private double profileTime = 0.0;

    public StaticProfileExecutor(StaticProfile profile, Output output, Input input, double allowableError) {
        this.profile = profile;
        this.output = output;
        this.input = input;
        this.allowableError = allowableError;

        finalTarget = profile.getPosition(profile.getDuration());
    }

    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        profileTime = 0.0;
    }

    // Returns true if error is within the allowable error, false otherwise
    public boolean update() {
        double time = Timer.getFPGATimestamp() - startTime;
        Setpoint sp = profile.getSetpoint(time);
        output.set(sp);

        double error = Math.abs(finalTarget - input.get());
        return error <= allowableError;
    }

    public void updateNoEnd() {
        profileTime += 0.02;
        Setpoint sp = profile.getSetpoint(profileTime);
        output.set(sp);
    }
}