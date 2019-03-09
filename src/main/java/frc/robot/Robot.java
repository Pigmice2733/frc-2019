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
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.pidf.Gains;
import frc.robot.pidf.PIDF;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Outtake;
import frc.robot.subsystems.Stingers;
import frc.robot.superstructure.Pose;
import frc.robot.superstructure.SuperStructure;
import frc.robot.utils.Bounds;
import frc.robot.utils.ButtonDebouncer;

public class Robot extends TimedRobot {
    Joystick driverJoystick;
    Joystick operatorJoystick;

    ButtonDebouncer modeToggle;
    boolean hatchMode = true;

    ButtonDebouncer climbToggle;
    boolean climbMode = false;

    boolean trimMode = false;
    double trimArmPose = 0.0;
    Vision vision;
    PIDF visionAlignment;
    boolean visionEnabled = false;

    Drivetrain drivetrain;
    Stingers stingers;
    AHRS navx;

    PowerDistributionPanel pdp;

    Elevator elevator;
    Arm arm;
    Intake intake;
    Manipulator manipulator;
    Outtake outtake;

    SuperStructure superStructure;
    Pose target;

    @Override
    public void robotInit() {
        // Gyro
        navx = new AHRS(SPI.Port.kMXP);

        // Drivetrain
        configureDrivetrain(3, 1, 4, 2);

        // Elevator
        TalonSRX elevatorWinch = new TalonSRX(5);
        TalonSRX elevatorFollower = new TalonSRX(6);
        elevatorWinch.setSensorPhase(false);
        elevatorWinch.setInverted(false);
        configureFollowerMotor(elevatorFollower, elevatorWinch);

        configCurrentLimit(elevatorWinch);
        configCurrentLimit(elevatorFollower);

        elevator = new Elevator(elevatorWinch);

        // Manipulator + Ball outtake
        manipulator = new Manipulator(new DoubleSolenoid(6, 7), new DoubleSolenoid(4, 5));
        outtake = new Outtake(new VictorSPX(8));

        // Arm
        TalonSRX shoulder = new TalonSRX(7);
        shoulder.setSensorPhase(true);
        arm = new Arm(shoulder);

        configCurrentLimit(shoulder);

        // Ball intake
        TalonSRX intakePivot = new TalonSRX(9);
        VictorSPX intakeFollower = new VictorSPX(10);
        TalonSRX intakeRoller = new TalonSRX(11);
        configureFollowerMotor(intakeFollower, intakePivot);
        intakeFollower.setInverted(InvertType.OpposeMaster);
        intake = new Intake(intakePivot, intakeRoller, navx);

        // Stinger pistons
        stingers = new Stingers(new DoubleSolenoid(2, 0), new DoubleSolenoid(3, 1));

        superStructure = new SuperStructure(elevator, arm, intake, stingers, navx);

        driverJoystick = new Joystick(0);
        operatorJoystick = new Joystick(1);

        modeToggle = new ButtonDebouncer(operatorJoystick, 9);
        climbToggle = new ButtonDebouncer(operatorJoystick, 10);

        vision = new Vision(this::isEnabled);
        Bounds visionOutputBounds = new Bounds(-0.4, 0.4);
        Gains alignmentGains = new Gains(0.2, 0.0, 0.0);
        visionAlignment = new PIDF(alignmentGains, visionOutputBounds);

        CameraServer server = CameraServer.getInstance();
        server.startAutomaticCapture("Driver Cam", 0);
    }

    @Override
    public void autonomousInit() {
        superStructure.initialize(SuperStructure.Target.HATCH_BOTTOM);
        vision.start();
    }

    @Override
    public void autonomousPeriodic() {
        gamePeriodic();
    }

    @Override
    public void teleopInit() {
        superStructure.initialize(superStructure.getPose());
    }

    @Override
    public void teleopPeriodic() {
        gamePeriodic();
    }

    @Override
    public void testInit() {
        vision.start();
    }

    @Override
    public void testPeriodic() {
        drivetrain.arcadeDrive(-driverJoystick.getY(), driverJoystick.getX());

        intake.setRoller(-driverJoystick.getY());

        if (operatorJoystick.getRawButton(3)) {
            if (!climbMode) {
                climbMode = true;
                intake.startBalancing();
            }
            intake.levelRobot();
        } else {
            climbMode = false;
            intake.drive(0);
        }

        elevator.updateSensor();
        elevator.drive(-operatorJoystick.getY() * 0.4);

        // arm.updateSensor();
        // arm.drive(-operatorJoystick.getRawAxis(5) * 0.5);

        arm.drive(0.0);

        intake.updateSensor();
        intake.drive(0.0);
        // intake.drive(-operatorJoystick.getRawAxis(5) * 0.6);

        // if (operatorJoystick.getRawButton(1)) {
        // outtake.drive(-0.15);
        // } else {
        // intake.setRoller(0.0);
        // }

        if (operatorJoystick.getRawButton(6)) {
            intake.setRoller(1.0);
        } else {
            intake.setRoller(0.0);
        }

        if (operatorJoystick.getRawButton(2)) {
            stingers.extend();
        } else if (operatorJoystick.getRawButton(5)) {
            stingers.retract();
        } else {
            stingers.stop();
        }
    }

    @Override
    public void disabledInit() {
        vision.stop();
    }

    @Override
    public void disabledPeriodic() {
        elevator.updateSensor();
        arm.updateSensor();
        intake.updateSensor();
    }

    private void gamePeriodic() {
        if (driverJoystick.getRawButton(1)) {
            double visionOffset = vision.getOffset();

            if (!visionEnabled) {
                visionEnabled = true;
                visionAlignment.initialize(visionOffset, Timer.getFPGATimestamp(), 0.0);
            }

            if (visionOffset != -1 || visionOffset != 0.0) {
                double output = visionAlignment.calculateOutput(visionOffset, 0.0, Timer.getFPGATimestamp());
                drivetrain.arcadeDrive(-driverJoystick.getY(), output);
            } else {
                System.out.println("Not connected/visible");
            }
        } else {
            visionEnabled = false;
            drivetrain.arcadeDrive(-driverJoystick.getY(), driverJoystick.getX());
        }

        if(operatorJoystick.getRawButton(8)) {
            if(!trimMode) {
                trimMode = true;
                trimArmPose = arm.getPosition();
            }
            arm.drive(-0.2 * operatorJoystick.getY());
            return;

        } else {
            if(trimMode) {
                arm.setPosition(trimArmPose);
                trimMode = false;
            }
        }

        if (modeToggle.get()) {
            hatchMode = !hatchMode;
        }

        if (hatchMode && !climbMode) {
            if (operatorJoystick.getRawButton(6)) {
                // right bumper
                manipulator.setPosition(Manipulator.State.Slack);
            } else if (operatorJoystick.getRawButton(5)) {
                // left bumper
                manipulator.setPosition(Manipulator.State.Extend);
            } else {
                manipulator.setPosition(Manipulator.State.Retract);
            }
            outtake.drive(0.0);
        } else if (!climbMode) {
            if (operatorJoystick.getRawButton(6)) {
                // right bumper
                intake.setRoller(0.6);
                outtake.drive(-0.4);
            } else if (operatorJoystick.getRawButton(5)) {
                // left bumper
                outtake.drive(0.6);
                intake.setRoller(0.0);
            } else {
                outtake.drive(-0.20);
                intake.setRoller(0.0);
            }
        }

        if (climbToggle.get()) {
            climbMode = !climbMode;
        }

        if (climbMode) {
            if (operatorJoystick.getRawButton(6)) {
                intake.levelRobot();
            } else {
                superStructure.initialize(SuperStructure.Target.PRE_CLIMB);
            }

            if (operatorJoystick.getRawButton(3)) {
                stingers.extend();
            } else if (operatorJoystick.getRawButton(4)) {
                stingers.retract();
            } else {
                stingers.stop();
            }

            intake.setRoller(-2.0 * driverJoystick.getY());
        } else {
            Pose target = findSetpoint();
            if (target != null) {
                superStructure.target(target);
            } else {
                superStructure.update();
            }

            stingers.retract();
        }
    }

    private Pose findSetpoint() {
        if (hatchMode) {
            if (operatorJoystick.getRawButton(1)) {
                // A
                return SuperStructure.Target.HATCH_BOTTOM;
            } else if (operatorJoystick.getRawButton(2)) {
                // B
                return SuperStructure.Target.HATCH_M_BACK;
            } else if (operatorJoystick.getRawButton(3)) {
                // X
                return SuperStructure.Target.HATCH_M_FRONT;
            } else if (operatorJoystick.getRawButton(4)) {
                // Y
                return SuperStructure.Target.HATCH_TOP;
            }
        } else {
            if (operatorJoystick.getRawButton(1)) {
                // A
                return SuperStructure.Target.CARGO_BOTTOM;
            } else if (operatorJoystick.getRawButton(2)) {
                // B
                return SuperStructure.Target.CARGO_M_BACK;
            } else if (operatorJoystick.getRawButton(3)) {
                // X
                return SuperStructure.Target.CARGO_M_FRONT;
            } else if (operatorJoystick.getRawButton(4)) {
                // Y
                return SuperStructure.Target.CARGO_TOP;
            }
        }

        if (operatorJoystick.getRawButton(7)) {
            return SuperStructure.Target.STARTING_CONFIG;
        }

        if (superStructure.getTarget().equals(SuperStructure.Target.CARGO_BOTTOM)
                || superStructure.getTarget().equals(SuperStructure.Target.CARGO_INTAKE)) {
            if (operatorJoystick.getRawButton(6)) {
                return SuperStructure.Target.CARGO_INTAKE;
            } else {
                return SuperStructure.Target.CARGO_BOTTOM;
            }
        }

        return null;
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
