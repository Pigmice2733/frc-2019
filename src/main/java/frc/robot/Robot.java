/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.IMotorControllerEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.autonomous.Autonomous;
import frc.robot.autonomous.Radius;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Selector;

public class Robot extends TimedRobot {
    Joystick joystick;
    Drivetrain drivetrain;
    AHRS navx;
    Selector<Autonomous> autoSelector;

    @Override
    public void robotInit() {
        AHRS navx = new AHRS(SPI.Port.kMXP);

        TalonSRX leftDrive = new TalonSRX(3);
        TalonSRX rightDrive = new TalonSRX(1);

        configureDriveMotor(leftDrive);
        configureDriveMotor(rightDrive);

        rightDrive.setInverted(true);

        VictorSPX leftFollower = new VictorSPX(4);
        VictorSPX rightFollower = new VictorSPX(2);

        configureFollowerMotor(leftFollower, leftDrive);
        configureFollowerMotor(rightFollower, rightDrive);

        drivetrain = new Drivetrain(leftDrive, rightDrive, navx, 2);

        joystick = new Joystick(0);

        autoSelector = new Selector<>("autonomous", "Drive Radius", new Radius(drivetrain, navx));
    }

    @Override
    public void autonomousInit() {
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
    }

    @Override
    public void teleopPeriodic() {
        drivetrain.arcadeDrive(-joystick.getY(), joystick.getX());
    }

    @Override
    public void disabledInit() {
    }

    private void configureDriveMotor(IMotorControllerEnhanced motor) {
        configureVoltageComp(motor);
        motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        motor.setSelectedSensorPosition(0, 0, 10);
    }

    private void configureFollowerMotor(IMotorController follower, IMotorController leader) {
        configureVoltageComp(follower);
        follower.follow(leader);
        follower.setInverted(leader.getInverted());
    }

    private void configureVoltageComp(IMotorController motor) {
        motor.configVoltageCompSaturation(11.0, 10);
        motor.enableVoltageCompensation(true);
        motor.configVoltageMeasurementFilter(32, 10);
    }
}
