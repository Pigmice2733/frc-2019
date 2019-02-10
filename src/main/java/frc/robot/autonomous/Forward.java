package frc.robot.autonomous;

import frc.robot.subsystems.Drivetrain;

import java.util.Arrays;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.autonomous.subroutines.Drive;

public class Forward extends Autonomous {
    private AHRS navx;

    public Forward(Drivetrain drive, AHRS navx) {
        this.navx = navx;
        this.subroutines = Arrays.asList(new Drive(drive, navx, 20.0, 0.0));
    }

    public void initialize() {
        navx.setAngleAdjustment(0.0);
        navx.setAngleAdjustment(-navx.getAngle());
    }
}
