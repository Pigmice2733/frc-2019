package com.pigmice.frc.robot.superstructure;

import com.kauailabs.navx.frc.AHRS;
import com.pigmice.frc.lib.logging.Logger;
import com.pigmice.frc.lib.utils.Range;
import com.pigmice.frc.lib.utils.Utils;
import com.pigmice.frc.robot.subsystems.Arm;
import com.pigmice.frc.robot.subsystems.Elevator;
import com.pigmice.frc.robot.subsystems.Intake;
import com.pigmice.frc.robot.subsystems.Stingers;

public class SuperStructure {
    private Elevator elevator;
    private Arm arm;
    private Intake intake;

    private Pose finalTarget;

    private static final Range intakeCollision = new Range(0.115, 0.47);

    private static Logger.ComponentLogger logger = Logger.createComponent(SuperStructure.class);

    private static String currentState = "0";
    public static void logState(String newState) {
        if (!newState.equals(currentState)) {
            logger.info(currentState + " -> " + newState);
            currentState = newState;
        }
    }

    public static class Target {
        public static final Pose STARTING_CONFIG = new Pose(0.138, Arm.Target.START, Intake.Target.START);
        public static final Pose HATCH_BOTTOM = new Pose(0.1, Arm.Target.DOWN_SLIGHT, Intake.Target.STOWED_BACK);
        public static final Pose HATCH_M_FRONT = new Pose(0.992, Arm.Target.DOWN_FLAT, Intake.Target.STOWED_BACK);
        public static final Pose HATCH_M_BACK = new Pose(0.0, Arm.Target.UP_ANGLE, Intake.Target.STOWED_BACK);
        public static final Pose HATCH_TOP = new Pose(0.875, Arm.Target.UP_FLAT, Intake.Target.STOWED_BACK);
        public static final Pose CARGO_BOTTOM = new Pose(0.1, Arm.Target.DOWN_SLIGHT, Intake.Target.STOWED_FRONT);
        public static final Pose CARGO_M_FRONT = new Pose(0.992, 0.27, Intake.Target.STOWED_UP);
        public static final Pose CARGO_SHIP = new Pose(0.9, Arm.Target.CARGO_SHIP, Intake.Target.STOWED_UP);
        public static final Pose CARGO_M_BACK = new Pose(0.0, 0.97, Intake.Target.STOWED_UP);
        public static final Pose CARGO_F_TOP = new Pose(0.98, 0.50, Intake.Target.STOWED_UP);
        public static final Pose CARGO_B_TOP = new Pose(0.825, 0.90, Intake.Target.STOWED_UP);
        public static final Pose CARGO_INTAKE = new Pose(0.02, Arm.Target.DOWN_UP, Intake.Target.INTAKE);
        public static final Pose CARGO_INTAKE_HIGH = new Pose(0.02, Arm.Target.DOWN_UP, Intake.Target.INTAKE - 0.075);
        public static final Pose CARGO_OUTTAKE_BOTTOM = new Pose(0.0, Arm.Target.CARGO_OUTTAKE, 0.52);
        public static final Pose PRE_CLIMB = new Pose(0.1, Arm.Target.CLIMB, Intake.Target.CLIMB);
    }

    public SuperStructure(Elevator elevator, Arm arm, Intake intake, Stingers stingers, AHRS navx) {
        this.elevator = elevator;
        this.arm = arm;
        this.intake = intake;
    }

    public void initialize(Pose target) {
        elevator.resetPID();
        arm.resetPID();
        target(target);
    }

    public void target(Pose target) {
        finalTarget = target;
        Pose currentPose = getPose();
        Pose currentTarget = getIntermediatePose(currentPose, target);

        elevator.setTargetPosition(currentTarget.elevator);
        arm.setTargetPosition(currentTarget.arm);
        intake.setTargetPosition(currentTarget.intake);

        elevator.update();
        arm.update();
        intake.update();
    }

    public void update() {
        target(finalTarget);
    }

    public Pose getTarget() {
        return finalTarget;
    }

    public Pose getPose() {
        return new Pose(elevator.getPosition(), arm.getPosition(), intake.getPosition());
    }

    /**
     * Returns whether an intermediate value is between a start and end position
     */
    private static boolean crosses(double start, double end, Range boundary) {
        Range range = new Range(start, end);
        return range.overlaps(boundary);
    }

    /**
     * Returns the next intermediate step in reaching the final target
     */
    public static Pose getIntermediatePose(Pose current, Pose target) {
        // Prevent ball intake from colliding with arm manipulators
        if (crosses(current.intake, target.intake, intakeCollision)) {
            // Intake could currently be above manipulators
            if (intakeCollision.contains(current.intake)) {
                // Arm starts down
                if (current.arm < 0.3 && current.elevator < 0.8 && (target.arm > 0.075 || target.elevator > 0.12
                        || (current.intake > 0.2 && target.intake < 0.1))) {
                    // move the intake out in front
                    if (current.intake < 0.3 && target.intake > 0.47) {
                        logState("1");
                        return current.setArmMin(0.4).setIntake(0.075);
                    } else {
                        logState("2");
                        return current.setIntakeMin(0.56);
                    }
                }

                // Arm is trying to swing down
                if (target.arm < 0.3 && target.elevator < 0.9 && (current.elevator > 0.12 || current.arm > 0.1)) {
                    if (intakeCollision.contains(target.intake)) {
                        if (current.intake < 0.475) {
                            logState("3");
                            return current.setIntakeMin(0.56);
                        } else {
                            logState("4");
                            return target.setIntakeMin(0.56).setArmMin(0.4).setElevatorMin(0.35);
                        }
                    } else {
                        if (current.intake < 0.475 && target.intake > 0.2 && target.arm < 0.2) {
                            logState("5");
                            return current.setIntakeMin(0.56);
                        } else {
                            logState("6");
                            return target.setArmMin(0.4).setElevatorMin(0.35);
                        }
                    }
                }
            }

            if (intakeCollision.contains(target.intake) && (current.arm < 0.3 || target.arm < 0.3)
                    && (current.elevator < 0.85 || target.elevator < 0.9)
                    && !Utils.almostEquals(target.arm, Arm.Target.DOWN_UP)) {
                if (current.intake < 0.1 && current.elevator < 0.3) {
                    logState("7");
                    return target.setElevatorMin(0.35).setArmMin(0.4).setIntake(current.intake);
                }

                if (current.intake < 0.54 && (current.elevator > 0.12 || current.arm > 0.1)) {
                    if (target.elevator > 0.875) {
                        logState("8");
                        return target.setIntakeMin(0.56).setElevatorMin(0.35).setArmMin(0.2);
                    } else {
                        if (current.arm < 0.3) {
                            logState("9");
                            return current.setElevatorMin(0.3).setArmMin(0.4);
                        } else if (current.intake > 0.465 && target.arm < 0.15) {
                            if (target.elevator < 0.01 && target.arm <= 0.11 && target.arm > 0.1) {
                                logState("10");
                                return target.setIntakeMin(0.56).setElevatorMin(0.05);
                            }
                            logState("11");
                            return target.setIntakeMin(0.56).setArmMin(current.arm);
                        } else {
                            logState("12");
                            return target.setIntakeMin(0.56).setElevatorMin(0.35).setArmMin(0.4);
                        }
                    }
                }

                if (current.elevator > 0.12 || current.arm > 0.1 || target.arm > 0.4 || target.elevator > 0.875) {
                    logState("13");
                    return target.setIntakeMin(0.56);
                }

                logState("14");
                return target;
            }

            if (current.elevator < 0.9 && !Utils.almostEquals(target.arm, Arm.Target.DOWN_UP)) {
                if (current.arm > 0.6 && target.arm > 0.6) {
                    logState("15");
                    return target;
                }

                if ((current.arm > 0.3 || (current.intake > 0.4 && current.elevator > 0.2)
                        || (current.arm > 0.1 && current.elevator > 0.8))
                        && (current.elevator > 0.15 || target.arm > 0.35 || target.elevator > 0.875)) {
                    if (target.arm > 0.7 || current.elevator > 0.8) {
                        logState("16");
                        return target;
                    } else {
                        logState("17");
                        return target.setElevatorMin(0.35).setArmMin(0.4);
                    }
                }

                logState("18");
                return current.setElevatorMin(0.35).setArmMin(0.4);
            }
        }

        if (target.intake >= 0.5 && Utils.almostEquals(target.arm, Arm.Target.CARGO_OUTTAKE)) {
            if (current.arm < (target.arm - 0.035)) {
                logState("19");
                return target.setIntakeMin(0.63).setElevatorMin(0.3);
            }
        }

        if (target.intake >= 0.5 && Utils.almostEquals(target.arm, Arm.Target.DOWN_UP)) {
            if (current.intake < 0.47) {
                logState("20");
                return current.setElevatorMin(0.5).setArmMin(0.4);
            }
            if (current.arm < (target.arm - 0.025)) {
                logState("21");
                return target.setElevatorMin(0.3);
            }
        }

        if (target.arm > 0.04 && target.intake > 0.2 && current.arm < 0.0425) {
            logState("22");
            return target.setElevatorMin(0.2);
        }

        // Enter starting config
        if (Utils.almostEquals(target.arm, Arm.Target.START)) {
            if (current.arm > 0.0) {
                if (current.elevator < 0.15 && current.arm > 0.015) {
                    logState("23");
                    return target.setElevatorMin(0.165).setArmMin(0.02);
                } else {
                    logState("24");
                    return target.setElevatorMin(0.165);
                }
            } else {
                if (current.elevator < 0.13) {
                    logState("25");
                    return current.setArmMin(0.025).setElevator(0.1);
                }
                logState("26");
                return target;
            }
        }

        // Exit starting config
        if (current.arm < 0.01) {
            if (current.elevator < 0.15) {
                logState("27");
                return current.setElevatorMin(0.2);
            } else {
                logState("28");
                return target.setElevatorMin(0.2);
            }
        }

        logState("29");
        return target;
    }
}
