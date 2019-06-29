package com.pigmice.frc.robot.autonomous.subroutines;

import com.kauailabs.navx.frc.AHRS;
import com.pigmice.frc.lib.motion.Setpoint;
import com.pigmice.frc.lib.motion.StaticProfile;
import com.pigmice.frc.lib.motion.execution.StaticProfileExecutor;
import com.pigmice.frc.lib.motion.execution.StaticSteeringController;
import com.pigmice.frc.lib.pidf.Gains;
import com.pigmice.frc.lib.pidf.PIDF;
import com.pigmice.frc.lib.utils.Range;
import com.pigmice.frc.robot.subsystems.Drivetrain;

public class TurnRadius implements ISubroutine {
    private StaticProfileExecutor executor;
    private final StaticSteeringController steering;

    private AHRS navx;
    private final Drivetrain drive;

    private double angleOffset = 0.0;
    private double targetAngle = 0.0;
    private double radius = 0.0;
    private boolean finished = false;
    private boolean absolute = false;

    public TurnRadius(Drivetrain drive, AHRS navx, double degrees, double radius) {
        this(drive, navx, degrees, radius, false);
    }

    public TurnRadius(Drivetrain drive, AHRS navx, double degrees, double radius, boolean absolute) {
        this.drive = drive;
        this.navx = navx;
        this.absolute = absolute;
        this.targetAngle = degrees;
        this.radius = radius;

        Gains steeringGains = new Gains(0.00005, 0.00035, 0.0);
        Range steeringBounds = new Range(-0.2, 0.2);
        PIDF steeringPID = new PIDF(steeringGains, steeringBounds);
        steering = new StaticSteeringController(this::getAngle, steeringPID);
    }

    public void initialize() {
        double initialAngle = 0.0;
        if (absolute) {
            initialAngle = getAngle();
            angleOffset = 0.0;
        } else {
            angleOffset = getAngle();
        }

        StaticProfile profile = new StaticProfile(0.0, initialAngle, targetAngle, 70.0, 60.0, 80.0);
        executor = new StaticProfileExecutor(profile, this::driveOutput, this::getAngle, 1);

        drive.initializePID();
        steering.initialize(initialAngle);
        executor.initialize();
        finished = false;
    }

    public boolean update() {
        if (!finished) {
            finished = executor.update();
        }
        return finished;
    }

    private double getAngle() {
        return -navx.getAngle() - angleOffset;
    }

    private void driveOutput(Setpoint sp) {
        Setpoint left = sp.toArcLength(radius + 0.5 * drive.getTrackWidth(), false);
        Setpoint right = sp.toArcLength(radius - 0.5 * drive.getTrackWidth(), false);

        double correction = steering.correct(sp.getHeading());
        left = new Setpoint(left.getPosition() - correction, left.getVelocity(), left.getAcceleration(),
                left.getCurvature(), left.getHeading());
        right = new Setpoint(right.getPosition() - correction, right.getVelocity(), right.getAcceleration(),
                right.getCurvature(), right.getHeading());

        drive.PIDDrive(left, right);
    }
}
