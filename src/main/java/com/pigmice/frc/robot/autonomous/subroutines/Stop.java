package com.pigmice.frc.robot.autonomous.subroutines;

import com.pigmice.frc.robot.subsystems.Drivetrain;

public class Stop implements ISubroutine {
    private final Drivetrain drive;

    public Stop(Drivetrain drive) {
        this.drive = drive;
    }

    public void initialize() {
    }

    public boolean update() {
        drive.stop();
        return false;
    }
}
