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
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Vision.Target;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.Stingers;
import frc.robot.superstructure.Pose;
import frc.robot.superstructure.SuperStructure;

public class Robot extends TimedRobot {
    Joystick driverJoystick;
    Joystick operatorJoystick;
    Vision vision;

    Drivetrain drivetrain;
    Stingers stingers;
    AHRS navx;

    Elevator elevator;
    Arm arm;
    Intake intake;
    Manipulator manipulator;
    Outtake outtake;

    SuperStructure superStructure;
    Pose target;

    @Override
    public void robotInit() {
        AHRS navx = new AHRS(SPI.Port.kMXP);

        configureDrivetrain(3, 1, 4, 2);

        TalonSRX elevatorWinch = new TalonSRX(5);
        TalonSRX elevatorFollower = new TalonSRX(6);
        elevatorWinch.setSensorPhase(true);
        elevatorWinch.setInverted(true);
        configureFollowerMotor(elevatorFollower, elevatorWinch);

        configCurrentLimit(elevatorWinch);
        configCurrentLimit(elevatorFollower);

        elevator = new Elevator(elevatorWinch);

        manipulator = new Manipulator(new DoubleSolenoid(0, 1), new DoubleSolenoid(2, 3));
        outtake = new Outtake(new VictorSPX(8));

        TalonSRX shoulder = new TalonSRX(7);
        shoulder.setSensorPhase(true);
        arm = new Arm(shoulder);

        configCurrentLimit(shoulder);

        TalonSRX intakePivot = new TalonSRX(9);
        intakePivot.setInverted(true);
        VictorSPX intakeFollower = new VictorSPX(10);
        TalonSRX intakeRoller = new TalonSRX(11);
        configureFollowerMotor(intakeFollower, intakePivot);
        intakeFollower.setInverted(InvertType.OpposeMaster);
        intake = new Intake(intakePivot, intakeRoller);

        stingers = new Stingers(new DoubleSolenoid(4, 5));

        superStructure = new SuperStructure(elevator, arm, intake, stingers, navx);

        driverJoystick = new Joystick(0);
        operatorJoystick = new Joystick(1);

        vision = new Vision(this::isEnabled);
        vision.start();

        CameraServer server = CameraServer.getInstance();
        server.startAutomaticCapture("Driver Cam", 0);
    }

    @Override
    public void teleopInit() {
        target = SuperStructure.Target.STARTING_CONFIG;
        superStructure.initialize(target);
    }

    @Override
    public void teleopPeriodic() {
        drivetrain.arcadeDrive(-driverJoystick.getY(), driverJoystick.getX());

        // if (operatorJoystick.getRawButton(4)) {
        // // Y
        // target = SuperStructure.Target.HATCH_TOP;
        // // elevator = 0.9;
        // // arm = Arm.Target.UP_FLAT;
        // } else if (operatorJoystick.getRawButton(2)) {
        // // B
        // target = SuperStructure.Target.HATCH_M_BACK;
        // // elevator = 0.08;
        // // arm = Arm.Target.UP_FLAT;
        // } else if (operatorJoystick.getRawButton(1)) {
        // // A
        // target = SuperStructure.Target.HATCH_BOTTOM;
        // // elevator = 0.0;
        // // arm = Arm.Target.DOWN_FLAT;
        // } else if (operatorJoystick.getRawButton(3)) {
        // // X
        // target = SuperStructure.Target.HATCH_M_FRONT;
        // // elevator = 1.1;
        // // arm = Arm.Target.DOWN_FLAT;
        // } else if (operatorJoystick.getRawButton(7)) {
        // target = SuperStructure.Target.STARTING_CONFIG;
        // }

        // if (operatorJoystick.getRawButton(6)) {
        // // right bumper
        // manipulator.setPosition(Manipulator.State.Slack);
        // } else if (operatorJoystick.getRawButton(5)) {
        // // left bumper
        // manipulator.setPosition(Manipulator.State.Extend);
        // } else {
        // manipulator.setPosition(Manipulator.State.Retract);
        // }

        // superStructure.setTarget(target);

        if (operatorJoystick.getRawButton(1)) {
            intake.setTargetPosition(Intake.Target.INTAKE);
        } else if (operatorJoystick.getRawButton(4)) {
            intake.setTargetPosition(Intake.Target.STOWED_UP);
        }

        intake.update();

        Target visionTarget = vision.getTarget();
        System.out.println("Offset: " + visionTarget.offset);
    }

    @Override
    public void testPeriodic() {
        elevator.updateSensor();
        elevator.drive(-operatorJoystick.getY() * 0.4);

        // arm.updateSensor();
        // arm.drive(-operatorJoystick.getRawAxis(5) * 0.5);

        intake.updateSensor();
        intake.drive(-operatorJoystick.getRawAxis(5) * 0.6);

        if (operatorJoystick.getRawButton(1)) {
            outtake.drive(-0.15);
        } else {
            intake.setRoller(0.0);
        }

        if (operatorJoystick.getRawButton(4)) {
            intake.setRoller(0.6);
        } else {
            intake.setRoller(0.0);
        }

    }

    @Override
    public void disabledPeriodic() {
        elevator.updateSensor();
        arm.updateSensor();
        intake.updateSensor();

        Target target = vision.getTarget();
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

    private void configCurrentLimit(IMotorControllerEnhanced motor) {
        motor.configContinuousCurrentLimit(10, 10);
        motor.configPeakCurrentLimit(20, 10);
        motor.configPeakCurrentDuration(500, 10);
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
