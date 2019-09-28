/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.pigmice.frc.robot;

import java.net.URI;
import java.net.URISyntaxException;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import com.pigmice.frc.lib.logging.Logger;
import com.pigmice.frc.lib.logging.Logger.ComponentLogger;
import com.pigmice.frc.robot.motorconfig.CTRE;
import com.pigmice.frc.robot.motorconfig.REV;
import com.pigmice.frc.robot.subsystems.Arm;
import com.pigmice.frc.robot.subsystems.AuxLighting;
import com.pigmice.frc.robot.subsystems.Drivetrain;
import com.pigmice.frc.robot.subsystems.Elevator;
import com.pigmice.frc.robot.subsystems.Intake;
import com.pigmice.frc.robot.subsystems.Manipulator;
import com.pigmice.frc.robot.subsystems.Stingers;
import com.pigmice.frc.robot.superstructure.Pose;
import com.pigmice.frc.robot.superstructure.SuperStructure;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.cscore.MjpegServer;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Robot extends TimedRobot {
    ControlScheme controls;

    boolean trimMode = false;
    double trimArmPose = 0.0;

    MjpegServer camServer;
    Vision vision;

    Drivetrain drivetrain;
    Stingers stingers;

    Elevator elevator;
    Arm arm;
    Intake intake;
    Manipulator manipulator;

    SuperStructure superStructure;
    AuxLighting lighting;

    ComponentLogger robotLogger = Logger.createComponent("Robot");

    @Override
    public void robotInit() {
        controls = new ControlScheme();

        AHRS navx = new AHRS(SPI.Port.kMXP);

        configureDrivetrain(3, 1, 4, 2, navx);

        configureElevator(5, 6);
        configureArm(7);

        boolean isIntakeInstalled = true;
        configureIntake(9, 10, navx, isIntakeInstalled);

        // Lobster + Ball outtake
        manipulator = new Manipulator(new DoubleSolenoid(6, 7), new DoubleSolenoid(4, 5), new VictorSPX(8),
                new AnalogInput(0));

        stingers = new Stingers(new DoubleSolenoid(2, 0), new DoubleSolenoid(3, 1));

        superStructure = new SuperStructure(elevator, arm, intake, stingers, navx);

        CameraServer server = CameraServer.getInstance();
        server.startAutomaticCapture("Driver Cam", 0);

        lighting = new AuxLighting(9);

        Vision.start();

        URI driverStation;
        try {
            driverStation = new URI("ws://10.27.33.5:8181/log");
        } catch (URISyntaxException e) {
            throw new RuntimeException("Misformatted driver station URI");
        }

        Logger.configure(driverStation);
        Logger.start();

        robotLogger.info("Robot code started");
    }

    @Override
    public void autonomousInit() {
        robotLogger.info("Autonomous init");

        superStructure.initialize(SuperStructure.Target.HATCH_BOTTOM);
        controls.initialize();

        if (DriverStation.getInstance().getAlliance() == Alliance.Red) {
            lighting.setBaseColor(AuxLighting.Color.RED);
        } else {
            lighting.setBaseColor(AuxLighting.Color.BLUE);
        }

        trimMode = false;
    }

    @Override
    public void autonomousPeriodic() {
        gamePeriodic();
    }

    @Override
    public void teleopInit() {
        robotLogger.info("Teleop init");

        superStructure.initialize(superStructure.getPose());
    }

    @Override
    public void teleopPeriodic() {
        gamePeriodic();
    }

    @Override
    public void testPeriodic() {
        drivetrain.arcadeDrive(controls.drive(), controls.steer());

        manipulator.drive(0);

        elevator.updateSensor();
        elevator.drive(-controls.operator.getY() * 0.4);

        arm.updateSensor();
        arm.drive(0.0);

        intake.updateSensor();
        intake.drive(-controls.operator.getRawAxis(5) * 0.35);

        if (controls.operator.getRawButton(5)) {
            intake.setRoller(0.5);
        } else {
            intake.setRoller(0.0);
        }

        if (controls.X()) {
            stingers.extend();
        } else if (controls.Y()) {
            stingers.retract();
        } else {
            stingers.stop();
        }
    }

    @Override
    public void disabledInit() {
        robotLogger.info("Disabled");
    }

    @Override
    public void disabledPeriodic() {
        elevator.updateSensor();
        arm.updateSensor();
        intake.updateSensor();
    }

    private void gamePeriodic() {
        elevator.updateSensor();
        arm.updateSensor();
        intake.updateSensor();

        controls.update();

        if (!controls.visionEngaged()) {
            drivetrain.arcadeDrive(controls.drive(), controls.steer());
        } else {
            drivetrain.visionDrive(controls.drive(), controls.steer(), Vision.targetVisible(), Vision.getOffset());
        }

        if (controls.hatchMode()) {
            if (controls.rightBumper()) {
                manipulator.setPosition(Manipulator.State.Slack);
            } else if (controls.leftBumper()) {
                manipulator.setPosition(Manipulator.State.Extend);
            } else {
                manipulator.setPosition(Manipulator.State.Retract);
            }

            manipulator.drive(0.0);
            intake.setRoller(0.0);
        } else if (!controls.climbMode()) {
            if (controls.rightBumper()) {
                intake.setRoller(0.8);
                manipulator.drive(-0.4);
            } else if (controls.leftBumper()) {
                manipulator.drive(0.6);
                intake.setRoller(0.0);
            } else {
                if (arm.getVelocity() > 0.0075) {
                    manipulator.drive(-0.40);
                } else {
                    manipulator.drive(-0.20);
                }
                intake.setRoller(0.0);
            }
        }

        if (controls.climbMode()) {
            if (controls.rightBumper()) {
                intake.levelRobot();
            } else if (controls.panicMode()) {
                double speed = 0.0;
                if (controls.driver.getRawButton(7)) {
                    speed = -0.15;
                } else if (controls.driver.getRawButton(8)) {
                    speed = 0.1;
                }
                intake.drive(speed);
                arm.setTargetPosition(0.0);
                arm.update();
                elevator.setTargetPosition(0.1);
                elevator.update();
            } else {
                superStructure.initialize(SuperStructure.Target.PRE_CLIMB);
            }

            if (controls.X()) {
                stingers.extend();
            } else if (controls.Y()) {
                stingers.retract();
            } else {
                stingers.stop();
            }

            intake.setRoller(2.0 * controls.drive());
            manipulator.drive(0.0);
        } else {
            Pose target = controls.findSetpoint();

            if (target == SuperStructure.Target.CARGO_BOTTOM && manipulator.hasBall()) {
                target = SuperStructure.Target.CARGO_OUTTAKE_BOTTOM;
            }

            if (target != null) {
                superStructure.target(target);
            } else {
                superStructure.update();
            }

            stingers.retract();
        }

        if (controls.trimMode()) {
            if (!trimMode) {
                trimMode = true;
                trimArmPose = arm.getPosition();
            }
            arm.drive(-0.2 * controls.operator.getY());
            return;
        } else {
            if (trimMode) {
                arm.setPosition(trimArmPose);
                trimMode = false;
            }
        }

        if (manipulator.hasBall()) {
            lighting.setColor(AuxLighting.Color.ORANGE);
        } else {
            lighting.resetToBase();
        }
    }

    private void configureDrivetrain(int frontLeft, int frontRight, int backLeft, int backRight, AHRS navx) {
        TalonSRX leftDrive = new TalonSRX(frontLeft), rightDrive = new TalonSRX(frontRight);
        CTRE.configureDriveMotor(leftDrive);
        CTRE.configureDriveMotor(rightDrive);

        rightDrive.setInverted(true);

        VictorSPX leftFollower = new VictorSPX(backLeft), rightFollower = new VictorSPX(backRight);

        CTRE.configureFollowerMotor(leftFollower, leftDrive);
        CTRE.configureFollowerMotor(rightFollower, rightDrive);

        drivetrain = new Drivetrain(leftDrive, rightDrive, navx, 2);
    }

    private void configureElevator(int encoder, int follower) {
        TalonSRX elevatorWinch = new TalonSRX(encoder);
        TalonSRX elevatorFollower = new TalonSRX(follower);

        elevatorWinch.setSensorPhase(false);
        elevatorWinch.setInverted(false);
        CTRE.configureFollowerMotor(elevatorFollower, elevatorWinch);

        elevatorFollower.setInverted(true);

        CTRE.configCurrentLimit(elevatorWinch);
        CTRE.configCurrentLimit(elevatorFollower);

        elevator = new Elevator(elevatorWinch);
    }

    private void configureArm(int shoulder) {
        TalonSRX motor = new TalonSRX(shoulder);
        motor.setSensorPhase(true);
        CTRE.configCurrentLimit(motor);

        arm = new Arm(motor);
    }

    private void configureIntake(int pivot, int follower, AHRS navx, boolean isIntakeInstalled) {
        if (isIntakeInstalled) {
            CANSparkMax intakePivot = new CANSparkMax(pivot, MotorType.kBrushless);
            CANSparkMax intakeFollower = new CANSparkMax(follower, MotorType.kBrushless);

            REV.configureNeo(intakePivot);
            REV.configureNeo(intakeFollower);

            TalonSRX intakeRoller = new TalonSRX(11);
            intakeRoller.configContinuousCurrentLimit(25, 10);
            intakeRoller.configPeakCurrentLimit(0, 10);
            intakeRoller.configPeakCurrentDuration(0, 10);
            intakeRoller.enableCurrentLimit(true);
            intake = new Intake(intakePivot, intakeFollower, intakePivot.getEncoder(), intakeRoller, navx);
        } else {
            intake = Intake.UninstalledIntake();
        }
    }
}
