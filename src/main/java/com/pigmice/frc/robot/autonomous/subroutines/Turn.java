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

public class Turn implements ISubroutine {
    private StaticProfileExecutor executor;
    private final StaticSteeringController steering;

    private AHRS navx;
    private final Drivetrain drive;

    private double angleOffset = 0.0;
    private double targetAngle = 0.0;
    private boolean finished = false;
    private boolean absolute = false;
    private double initialAngle = 0.0;

    public Turn(Drivetrain drive, AHRS navx, double degrees) {
        this(drive, navx, degrees, false);
    }

    public Turn(Drivetrain drive, AHRS navx, double degrees, boolean absolute) {
        this.drive = drive;
        this.navx = navx;
        this.absolute = absolute;
        this.targetAngle = degrees;

        Gains steeringGains = new Gains(0.0008, 0.0007, 0.0);
        Range steeringBounds = new Range(-0.2, 0.2);
        PIDF steeringPID = new PIDF(steeringGains, steeringBounds);
        steering = new StaticSteeringController(this::getAngle, steeringPID);
    }

    public void initialize() {
        initialAngle = 0.0;
        if (absolute) {
            initialAngle = getAngle();
        }
        angleOffset = getAngle();

        StaticProfile profile = new StaticProfile(0.0, 0.0, targetAngle - initialAngle, 360.0, 300.0, 160.0);
        executor = new StaticProfileExecutor(profile, this::driveOutput, this::getAngle, 1);

        drive.initializePID();
        steering.initialize(0.0);
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
        sp = sp.toArcLength(0.5 * drive.getTrackWidth(), false);

        double correction = steering.correct(sp.getHeading());
        sp = new Setpoint(sp.getPosition() + correction, sp.getVelocity(), sp.getAcceleration(), sp.getCurvature(),
                sp.getHeading());

        drive.PIDDrive(sp.negate(false), sp);
    }
}
