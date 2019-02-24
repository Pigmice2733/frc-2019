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

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.Stingers;

public class Robot extends TimedRobot {
    Joystick driverJoystick;
    Joystick operatorJoystick;

    Drivetrain drivetrain;
    Stingers stingers;
    AHRS navx;

    Elevator elevator;
    Arm arm;
    Manipulator manipulator;
    Outtake outtake;

    @Override
    public void robotInit() {
        AHRS navx = new AHRS(SPI.Port.kMXP);

        configureDrivetrain(3, 1, 4, 2);

        TalonSRX elevatorWinch = new TalonSRX(5);
        TalonSRX elevatorFollower = new TalonSRX(6);
        elevatorWinch.setSensorPhase(true);
        elevatorWinch.setInverted(true);
        configureFollowerMotor(elevatorFollower, elevatorWinch);

        elevator = new Elevator(elevatorWinch);

        manipulator = new Manipulator(new DoubleSolenoid(0, 1), new DoubleSolenoid(2, 3));
        outtake = new Outtake(new VictorSPX(8));

        TalonSRX shoulder = new TalonSRX(7);
        shoulder.setSensorPhase(true);
        arm = new Arm(shoulder);

        stingers = new Stingers(new DoubleSolenoid(4, 5));

        driverJoystick = new Joystick(0);
        operatorJoystick = new Joystick(1);

        CameraServer server = CameraServer.getInstance();
        server.startAutomaticCapture("Driver Cam", 0);
    }

    @Override
    public void teleopPeriodic() {
        drivetrain.arcadeDrive(-driverJoystick.getY(), driverJoystick.getX());

        if (operatorJoystick.getRawButton(4)) {
            // Y
            elevator.setTargetPosition(0.9);
            arm.setTargetPosition(Arm.Target.UP_FLAT);
        } else if (operatorJoystick.getRawButton(2)) {
            // B
            elevator.setTargetPosition(0.08);
            arm.setTargetPosition(Arm.Target.UP_FLAT);
        } else if (operatorJoystick.getRawButton(1)) {
            // A
            elevator.setTargetPosition(0.0);
            arm.setTargetPosition(Arm.Target.DOWN_FLAT);
        } else if (operatorJoystick.getRawButton(3)) {
            // X
            elevator.setTargetPosition(1.1);
            arm.setTargetPosition(Arm.Target.DOWN_FLAT);
        }

        // outtake.drive(-0.3);

        // if (operatorJoystick.getRawButton(2)) {
        // stingers.fire();
        // } else {
        // stingers.retract();
        // }

        if (operatorJoystick.getRawButton(6)) {
            // right bumper
            manipulator.setPosition(Manipulator.State.Slack);
        } else if (operatorJoystick.getRawButton(5)) {
            // left bumper
            manipulator.setPosition(Manipulator.State.Extend);
        } else {
            manipulator.setPosition(Manipulator.State.Retract);
        }

        elevator.update();
        arm.update();
    }

    @Override
    public void testPeriodic() {
        elevator.updateSensor();
        elevator.drive(-operatorJoystick.getY() * 0.4);

        arm.updateSensor();
        arm.drive(-operatorJoystick.getRawAxis(5) * 0.5);

        if (operatorJoystick.getRawButton(1)) {
            outtake.drive(0.4);
        }
    }

    @Override
    public void disabledPeriodic() {
        elevator.updateSensor();
        arm.updateSensor();
    }

    private void configureDrivetrain(int frontLeft, int frontRight, int backLeft, int backRight) {
        TalonSRX leftDrive = new TalonSRX(frontLeft);
        TalonSRX rightDrive = new TalonSRX(frontRight);

        configureDriveMotor(leftDrive);
        configureDriveMotor(rightDrive);

        rightDrive.setInverted(true);

        VictorSPX leftFollower = new VictorSPX(backLeft);
        VictorSPX rightFollower = new VictorSPX(backRight);

        configureFollowerMotor(leftFollower, leftDrive);
        configureFollowerMotor(rightFollower, rightDrive);

        drivetrain = new Drivetrain(leftDrive, rightDrive, navx, 2);
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
