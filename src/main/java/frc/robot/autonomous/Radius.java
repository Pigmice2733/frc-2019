package frc.robot.autonomous;

import frc.robot.subsystems.Drivetrain;

import java.util.Arrays;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.autonomous.subroutines.Drive;
import frc.robot.autonomous.subroutines.TurnRadius;

public class Radius extends Autonomous {
    private AHRS navx;

    public Radius(Drivetrain drive, AHRS navx) {
        this.navx = navx;
        this.subroutines = Arrays.asList(new Drive(drive, navx, 20.0, 0.0), new TurnRadius(drive, navx, -180, 4.0),
                new Drive(drive, navx, 15.0, -180.0));
    }

    public void initialize() {
        navx.setAngleAdjustment(0.0);
        navx.setAngleAdjustment(-navx.getAngle());
    }
}
