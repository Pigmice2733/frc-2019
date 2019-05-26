package frc.robot.autonomous.subroutines;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.motion.Setpoint;
import frc.robot.motion.StaticProfile;
import frc.robot.motion.execution.StaticProfileExecutor;
import frc.robot.motion.execution.StaticSteeringController;
import frc.robot.pidf.Gains;
import frc.robot.pidf.PIDF;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Range;

public class Drive implements ISubroutine {

    private final StaticProfileExecutor executor;
    private final Drivetrain drive;
    private final StaticSteeringController steering;
    private AHRS navx;

    private boolean finished = false;
    private boolean absoluteHeading = false;

    public Drive(Drivetrain drive, AHRS navx, double feet) {
        this.drive = drive;
        this.navx = navx;
        StaticProfile profile;
        if (feet > 10) {
            profile = new StaticProfile(0.0, 0.0, feet, 5.0, 6.5, 5.5);
        } else {
            profile = new StaticProfile(0.0, 0.0, feet, 6.0, 4.0, 3.0);
        }
        executor = new StaticProfileExecutor(profile, this::driveOutput, drive::getSensorPosition, 0.05);

        Gains steeringGains = new Gains(0.001, 0.00004, 0.0);
        Range steeringBounds = new Range(-0.2, 0.2);
        PIDF steeringPID = new PIDF(steeringGains, steeringBounds);
        steering = new StaticSteeringController(this::getAngle, steeringPID);
    }

    public Drive(Drivetrain drive, AHRS navx, double feet, double targetAngle) {
        this.drive = drive;
        this.navx = navx;
        StaticProfile profile;
        if (feet > 10) {
            profile = new StaticProfile(0.0, 0.0, feet, 5.0, 6.5, 5.5);
        } else {
            profile = new StaticProfile(0.0, 0.0, feet, 6.0, 4.0, 3.0);
        }
        executor = new StaticProfileExecutor(profile, this::driveOutput, drive::getSensorPosition, 0.05);

        absoluteHeading = true;

        Gains steeringGains = new Gains(0.001, 0.0004, 0.0);
        Range steeringBounds = new Range(-0.2, 0.2);
        PIDF steeringPID = new PIDF(steeringGains, steeringBounds);
        steering = new StaticSteeringController(this::getAngle, steeringPID, targetAngle);
    }

    public void initialize() {
        drive.initializePID();
        if (!absoluteHeading) {
            steering.initialize(getAngle());
        } else {
            steering.initialize();
        }
        executor.initialize();
        finished = false;
    }

    private double getAngle() {
        return -navx.getAngle();
    }

    public boolean update() {
        if (!finished) {
            finished = executor.update();
        }
        return finished;
    }

    private void driveOutput(Setpoint sp) {
        double correction = steering.correct();
        Setpoint left = new Setpoint(sp.getPosition() - correction, sp.getVelocity(), sp.getAcceleration(),
                sp.getCurvature(), sp.getHeading());
        Setpoint right = new Setpoint(sp.getPosition() + correction, sp.getVelocity(), sp.getAcceleration(),
                sp.getCurvature(), sp.getHeading());
        drive.PIDDrive(left, right);
    }
}
