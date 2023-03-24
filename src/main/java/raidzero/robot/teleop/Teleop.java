package raidzero.robot.teleop;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import raidzero.robot.Constants.ArmConstants;
import raidzero.robot.submodules.Arm;
import raidzero.robot.submodules.Intake;
import raidzero.robot.submodules.Swerve;
import raidzero.robot.submodules.Wrist;
import raidzero.robot.utils.JoystickUtils;
import raidzero.robot.utils.AutoAimController.AutoAimLocation;

public class Teleop {

    private static Teleop instance = null;
    private static XboxController p1 = new XboxController(0);
    private static XboxController p2 = new XboxController(1);
    private static GenericHID p3 = new GenericHID(2);

    private static final Arm arm = Arm.getInstance();
    private static final Swerve swerve = Swerve.getInstance();
    private static final Wrist wrist = Wrist.getInstance();
    private static final Intake intake = Intake.getInstance();

    private Alliance alliance;
    private boolean blue = false;
    private double reverse = 1; // joystick reverse

    public static Teleop getInstance() {
        if (instance == null) {
            instance = new Teleop();
        }
        return instance;
    }

    public void onStart() {
        blue = DriverStation.getAlliance() == Alliance.Blue;
        reverse = blue ? 1 : -1;

        // TEMPORARY

    }

    // private Timer mTimer = new Timer();
    // private double p1Time = 0.0;
    // private double p3Time = 0.0;
    public void onLoop() {
        /**
         * p1 controls
         */
        // mTimer.restart();
        p1Loop(p1);
        // p1Time = mTimer.get();
        /**
         * p2 controls
         */
        // p2Loop(p2);
        /**
         * p3 controls
         */
        // mTimer.restart();
        p3Loop(p3);
        // p3Time = mTimer.get();

        // if(p1Time + p3Time > 0.02) {
        // System.out.println("P1 Time :: " + p1Time);
        // System.out.println("P3 Time :: " + p3Time);
        // System.out.println();
        // }
    }

    private double[] target = { 0, 0.15 };

    private boolean aiming = false;
    private boolean fIntake = false;
    private boolean noSafenoProblemo = false;

    private void p1Loop(XboxController p) {
        SmartDashboard.putBoolean("Aiming", aiming);
        SmartDashboard.putBoolean("Safety", noSafenoProblemo);

        if (p.getYButton()) {
            aiming = true;
            // swerve.lockTo90();
        }
        if (p.getBButtonPressed()) {
            aiming = false;
            swerve.emptyBucket();
        }

        // if (p.getAButtonPressed()) {
        // noSafenoProblemo = !noSafenoProblemo && !p.getAButtonPressed();
        // }

        if (p.getXButtonPressed()) {
            swerve.zeroTele(blue ? 180 : 0);
            // swerve.zero();
        }

        if (p.getAButton()) {
            intake.setPercentSpeed(1.0);
        }

        // if (p.getAButton() && Math.abs(swerve.getBeans()) < 20) {
        // swerve.drive(0.2, 0, 0, true);
        // } else {
        if (!aiming)
            swerve.drive(
                    JoystickUtils.deadband(-p.getLeftY() * arm.tooFasttooFurious() *
                            arm.slurping() * reverse),
                    JoystickUtils.deadband(-p.getLeftX() * arm.tooFasttooFurious() *
                            arm.slurping() * reverse),
                    JoystickUtils.deadband(-p.getRightX() * arm.tooFasttooFurious() *
                            arm.slurping()),
                    true);
        else
            swerve.drive(
                    JoystickUtils.aimingDeadband(-p.getLeftY() * 0.25 * reverse),
                    JoystickUtils.aimingDeadband(-p.getLeftX() * 0.25 * reverse),
                    JoystickUtils.aimingDeadband(-p.getRightX()),
                    true);
        // }

        if (p.getLeftBumper() && !p.getRightBumper() && !arm.atPosition(ArmConstants.INTER_REV_CUBE_FLOOR_INTAKE, false)
                && !arm.atPosition(ArmConstants.REV_CUBE_FLOOR_INTAKE, false)) {
            fIntake = true;
            arm.moveToPoint(
                    ArmConstants.CUBE_DUMP, true);
        }
        if (p.getRightBumper() && !p.getLeftBumper() && !arm.atPosition(ArmConstants.CUBE_DUMP, true)) {
            fIntake = true;
            arm.moveTwoPronged(
                    ArmConstants.INTER_REV_CUBE_FLOOR_INTAKE,
                    ArmConstants.REV_CUBE_FLOOR_INTAKE, false);
            intake.setPercentSpeed(-0.7);
        } else if (arm.isSafe()) {
            fIntake = false;
        } else if (fIntake && !p.getRightBumper() && !p.getLeftBumper()) {
            arm.goHome();
            intake.setPercentSpeed(-0.7);
        }

        // double xSpeed = -p.getLeftY() * reverse * (aiming ? arm.tooFasttooFurious() *
        // arm.slurping() : 0.25);
        // double ySpeed = -p.getLeftX() * reverse * (aiming ? arm.tooFasttooFurious() *
        // arm.slurping() : 0.25);
        // double angularSpeed = -p.getRightX() * (aiming ? arm.tooFasttooFurious() *
        // arm.slurping() * 2.0 : 0.5);
        // swerve.drive(JoystickUtils.deadband(xSpeed), JoystickUtils.deadband(ySpeed),
        // JoystickUtils.deadband(angularSpeed), true);

        // // Auto Alignments
        // if (blue && ((!arm.isGoingHome() && arm.isSafe())
        // || noSafenoProblemo)) {
        // if (p.getRightBumper())
        // swerve.autoAim(AutoAimLocation.BR_LOAD);
        // else if (p.getLeftBumper())
        // swerve.autoAim(AutoAimLocation.BL_LOAD);

        // } else if (!blue && ((!arm.isGoingHome() && arm.isSafe())
        // || noSafenoProblemo)) {
        // if (p.getRightBumper())
        // swerve.autoAim(AutoAimLocation.RR_LOAD);
        // else if (p.getLeftBumper())
        // swerve.autoAim(AutoAimLocation.RL_LOAD);
        // }

        // if (p.getAButtonPressed())
        // p.setRumble(RumbleType.kBothRumble,0.0 );
    }

    private int mode = 0;

    private void p2Loop(XboxController p) {
        if (p.getRightBumperPressed())
            mode = 1; // Joystick
        else if (p.getBackButtonPressed())
            mode = 2; // Setpoints
        else if (p.getStartButtonPressed())
            mode = 3; // Joystick with Inv Kin.
        else if (p.getLeftBumperPressed())
            mode = 4; // Go Home

        if (mode == 1) {
            arm.moveArm(p.getLeftX() * 0.2, p.getRightX() * 0.2);
            wrist.setPercentSpeed(p.getLeftY() * 0.2);
        } else if (mode == 2) {
            // Human Pickup Station
            if (p.getYButtonPressed() && !swerve.isOverLimit() && !arm.isGoingHome() &&
                    arm.isOnTarget()
                    && arm.isSafe()) {
                arm.configSmartMotionConstraints(
                        ArmConstants.LOWER_MAX_VEL * 1.5,
                        ArmConstants.LOWER_MAX_ACCEL * 1.5,
                        ArmConstants.UPPER_MAX_VEL * 0.75,
                        ArmConstants.UPPER_MAX_ACCEL * 0.75);

                arm.moveThreePronged(ArmConstants.INTER_HUMAN_PICKUP_STATION,
                        ArmConstants.INTER2_HUMAN_PICKUP_STATION,
                        ArmConstants.HUMAN_PICKUP_STATION, true);
            }
            // High Grid
            else if (p.getBButtonPressed() &&
                    !swerve.isOverLimit() &&
                    !arm.isGoingHome() &&
                    arm.isOnTarget() &&
                    arm.isSafe()) {
                arm.moveTwoPronged(
                        ArmConstants.INTER_GRID_HIGH,
                        ArmConstants.GRID_HIGH,
                        true);
            }
            // Medium Grid
            else if (p.getAButtonPressed() &&
                    !swerve.isOverLimit() &&
                    !arm.isGoingHome() &&
                    arm.isOnTarget() &&
                    arm.isSafe()) {
                arm.moveTwoPronged(
                        ArmConstants.INTER_GRID_MEDIUM,
                        ArmConstants.GRID_MEDIUM, true);
            }
            // Floor Intake
            else if (p.getXButtonPressed() &&
                    !swerve.isOverLimit() &&
                    !arm.isGoingHome() &&
                    arm.isOnTarget() &&
                    arm.isSafe()) {
                arm.moveTwoPronged(
                        ArmConstants.INTER_FLOOR_INTAKE,
                        ArmConstants.FLOOR_INTAKE, true);
            }

        } else if (mode == 3) {
            // Extension Limits
            if (Math.abs(target[0]) <= ArmConstants.X_EXTENSION_LIMIT && target[1] <= ArmConstants.Y_EXTENSION_LIMIT
                    && target[1] >= 0) {
                target[0] = arm.getState()[1].getX() + MathUtil.applyDeadband(p.getLeftX() *
                        0.25, 0.05);
                target[1] = arm.getState()[1].getY() + MathUtil.applyDeadband(p.getRightY() *
                        -0.25, 0.05);
            }
            // Soft Joystick Limits
            else if (Math.abs(target[0]) > ArmConstants.X_EXTENSION_LIMIT) {
                if (Math.signum(target[0]) == -1)
                    target[0] = -ArmConstants.X_EXTENSION_LIMIT;
                else
                    target[0] = ArmConstants.X_EXTENSION_LIMIT;
            } else if (target[1] > ArmConstants.Y_EXTENSION_LIMIT)
                target[1] = ArmConstants.Y_EXTENSION_LIMIT;
            else if (target[1] < 0)
                target[1] = 0;

            arm.moveToPoint(target);
        } else if (mode == 4) {
            arm.goHome();
            mode = 0;
        }

        // // Intake
        // if (Math.abs(p.getRightTriggerAxis() - p.getLeftTriggerAxis()) >= 0.2) {
        // intake.setPercentSpeed(p.getRightTriggerAxis() - p.getLeftTriggerAxis());
        // } else {
        // intake.holdPosition();
        // }
    }

    boolean buttonPressed = false;
    boolean wasPreviouslyPressed = false;

    private double dHumanPickup = 0;
    private double dHighDelivery = 0;
    private double dMediumDelivery = 0;

    private void p3Loop(GenericHID p) {
        // Human Pickup Station
        if (p.getRawButtonPressed(10) &&
                ((!swerve.isOverLimit() && !arm.isGoingHome() && arm.isOnTarget() && arm.isSafe() && !fIntake)
                        || noSafenoProblemo)) {
            // Safe Human Pickup
            // arm.configSmartMotionConstraints(
            // ArmConstants.LOWER_MAX_VEL * 1.5,
            // ArmConstants.LOWER_MAX_ACCEL * 1.5,
            // ArmConstants.UPPER_MAX_VEL * 0.75,
            // ArmConstants.UPPER_MAX_ACCEL * 0.75);

            // arm.moveThreePronged(
            // ArmConstants.INTER_HUMAN_PICKUP_STATION,
            // ArmConstants.INTER2_HUMAN_PICKUP_STATION,
            // ArmConstants.HUMAN_PICKUP_STATION, true);

            // Extended Human Pickup
            arm.moveTwoPronged(
                    ArmConstants.INTER_EXT_HUMAN_PICKUP_STATION,
                    new double[] { ArmConstants.EXT_HUMAN_PICKUP_STATION[0],
                            ArmConstants.EXT_HUMAN_PICKUP_STATION[1] + dHumanPickup,
                            ArmConstants.EXT_HUMAN_PICKUP_STATION[2] },
                    false);
        } else if (p1.getBackButtonPressed() && arm.atPosition(new double[] { ArmConstants.EXT_HUMAN_PICKUP_STATION[0],
                ArmConstants.EXT_HUMAN_PICKUP_STATION[1] + dHumanPickup, ArmConstants.EXT_HUMAN_PICKUP_STATION[2] },
                false)) {
            dHumanPickup -= 0.01;
            // Extended Human Pickup
            arm.moveTwoPronged(
                    ArmConstants.INTER_EXT_HUMAN_PICKUP_STATION,
                    new double[] { ArmConstants.EXT_HUMAN_PICKUP_STATION[0],
                            ArmConstants.EXT_HUMAN_PICKUP_STATION[1] + dHumanPickup,
                            ArmConstants.EXT_HUMAN_PICKUP_STATION[2] },
                    false);
        } else if (p1.getStartButtonPressed() && arm.atPosition(new double[] { ArmConstants.EXT_HUMAN_PICKUP_STATION[0],
                ArmConstants.EXT_HUMAN_PICKUP_STATION[1] + dHumanPickup, ArmConstants.EXT_HUMAN_PICKUP_STATION[2] },
                false)) {
            dHumanPickup += 0.01;
            // Extended Human Pickup
            arm.moveTwoPronged(
                    ArmConstants.INTER_EXT_HUMAN_PICKUP_STATION,
                    new double[] { ArmConstants.EXT_HUMAN_PICKUP_STATION[0],
                            ArmConstants.EXT_HUMAN_PICKUP_STATION[1] + dHumanPickup,
                            ArmConstants.EXT_HUMAN_PICKUP_STATION[2] },
                    false);
        }

        // High Grid
        else if (p.getRawButtonPressed(14) &&
                ((!swerve.isOverLimit() && !arm.isGoingHome() && arm.isOnTarget() && arm.isSafe() && !fIntake)
                        || noSafenoProblemo)) {
            // Cone
            arm.moveTwoPronged(
                    ArmConstants.INTER_GRID_HIGH,
                    new double[] { ArmConstants.GRID_HIGH[0],
                            ArmConstants.GRID_HIGH[1] + dHighDelivery,
                            ArmConstants.GRID_HIGH[2] },
                    true);

            // Rev Cone
            // arm.moveTwoPronged(
            // ArmConstants.INTER_REV_GRID_HIGH,
            // ArmConstants.REV_GRID_HIGH,
            // false);

            // Cube
            // arm.moveTwoPronged(
            // ArmConstants.INTER_CUBE_GRID_HIGH,
            // ArmConstants.CUBE_GRID_HIGH,
            // true);
        }

        else if (p1.getBackButtonPressed() && arm.atPosition(new double[] { ArmConstants.GRID_HIGH[0],
            ArmConstants.GRID_HIGH[1] + dHumanDelivery, ArmConstants.GRID_HIGH[2] },
            false)) {
        dHumanDelivery -= 0.01;
        // Extended Human Pickup
        arm.moveTwoPronged(
                ArmConstants.INTER_EXT_HUMAN_PICKUP_STATION,
                new double[] { ArmConstants.EXT_HUMAN_PICKUP_STATION[0],
                        ArmConstants.EXT_HUMAN_PICKUP_STATION[1] + dHumanPickup,
                        ArmConstants.EXT_HUMAN_PICKUP_STATION[2] },
                false);
    } else if (p1.getStartButtonPressed() && arm.atPosition(new double[] { ArmConstants.EXT_HUMAN_PICKUP_STATION[0],
            ArmConstants.EXT_HUMAN_PICKUP_STATION[1] + dHumanPickup, ArmConstants.EXT_HUMAN_PICKUP_STATION[2] },
            false)) {
        dHumanPickup += 0.01;
        // Extended Human Pickup
        arm.moveTwoPronged(
                ArmConstants.INTER_EXT_HUMAN_PICKUP_STATION,
                new double[] { ArmConstants.EXT_HUMAN_PICKUP_STATION[0],
                        ArmConstants.EXT_HUMAN_PICKUP_STATION[1] + dHumanPickup,
                        ArmConstants.EXT_HUMAN_PICKUP_STATION[2] },
                false);
    }
        // Medium Grid
        else if (p.getRawButtonPressed(15) &&
                ((!swerve.isOverLimit() && !arm.isGoingHome() && arm.isOnTarget() && arm.isSafe() && !fIntake)
                        || noSafenoProblemo)) {
            arm.moveTwoPronged(
                    ArmConstants.INTER_GRID_MEDIUM,
                    ArmConstants.GRID_MEDIUM, true);
        }
        // Floor Intake
        else if (p.getRawButtonPressed(16) &&
                ((!swerve.isOverLimit() && !arm.isGoingHome() && arm.isOnTarget() && arm.isSafe() && !fIntake)
                        || noSafenoProblemo)) {
            // Cube Scoop
            // arm.moveThreePronged(
            // ArmConstants.INTER_FLOOR_INTAKE,
            // ArmConstants.INTER2_FLOOR_INTAKE,
            // ArmConstants.FLOOR_INTAKE, false);

            //
            // ed(
            // ArmConstants.INTER_REV_CUBE_FLOOR_INTAKE,
            // ArmConstants.REV_CUBE_FLOOR_INTAKE, false);

            // Flipped Cone
            arm.moveTwoPronged(
                    ArmConstants.INTER_REV_FLIPPED_CONE_FLOOR_INTAKE,
                    ArmConstants.REV_FLIPPED_CONE_FLOOR_INTAKE, false);

            // Upright Cone
            // arm.moveToPoint(ArmConstants.REV_CONE_FLOOR_INTAKE, false);
        }
        // Reverse Stage
        else if (p.getRawAxis(0) == 1 &&
                ((!swerve.isOverLimit() && !arm.isGoingHome() && !arm.isSafe() && !fIntake)
                        || noSafenoProblemo)) {
            arm.reverseStage();
        }
        // Go Home
        else if (p.getRawButtonPressed(13)) {
            arm.goHome();
        }

        // Intake
        if (p.getRawButton(12)) {
            intake.setPercentSpeed(0.7);
        } else if (p.getRawButton(11)) {
            intake.setPercentSpeed(-0.7);
        } else if (!p1.getLeftBumper() && !p1.getAButton() && !p1.getRightBumper() && !p.getRawButton(12)
                && !p.getRawButton(11)) {
            intake.holdPosition();
        }

        // Auto Alignments
        if (((!arm.isGoingHome() && arm.isSafe())
                || noSafenoProblemo)) {
            if (p.getRawButton(9)) {
                buttonPressed = true;
                if (buttonPressed && !wasPreviouslyPressed) {
                    wasPreviouslyPressed = true;
                    swerve.setAutoAimLocation(AutoAimLocation.LL);
                    swerve.enableAutoAimController(true);
                }
                if (!buttonPressed) {
                    wasPreviouslyPressed = false;
                    swerve.enableAutoAimController(false);
                }
            } else if (p.getRawButton(8)) {
                buttonPressed = true;
                if (buttonPressed && !wasPreviouslyPressed) {
                    wasPreviouslyPressed = true;
                    swerve.setAutoAimLocation(AutoAimLocation.LM);
                    swerve.enableAutoAimController(true);
                }
                if (!buttonPressed) {
                    wasPreviouslyPressed = false;
                    swerve.enableAutoAimController(false);
                }
            } else if (p.getRawButton(7)) {
                buttonPressed = true;
                if (buttonPressed && !wasPreviouslyPressed) {
                    wasPreviouslyPressed = true;
                    swerve.setAutoAimLocation(AutoAimLocation.LR);
                    swerve.enableAutoAimController(true);
                }
                if (!buttonPressed) {
                    wasPreviouslyPressed = false;
                    swerve.enableAutoAimController(false);
                }
            } else if (p.getRawButton(6)) {
                buttonPressed = true;
                if (buttonPressed && !wasPreviouslyPressed) {
                    wasPreviouslyPressed = true;
                    swerve.setAutoAimLocation(AutoAimLocation.ML);
                    swerve.enableAutoAimController(true);
                }
                if (!buttonPressed) {
                    wasPreviouslyPressed = false;
                    swerve.enableAutoAimController(false);
                }
            } else if (p.getRawButton(5)) {
                buttonPressed = true;
                if (buttonPressed && !wasPreviouslyPressed) {
                    wasPreviouslyPressed = true;
                    swerve.setAutoAimLocation(AutoAimLocation.MM);
                    swerve.enableAutoAimController(true);
                }
                if (!buttonPressed) {
                    wasPreviouslyPressed = false;
                    swerve.enableAutoAimController(false);
                }
            } else if (p.getRawButton(4)) {
                buttonPressed = true;
                if (buttonPressed && !wasPreviouslyPressed) {
                    wasPreviouslyPressed = true;
                    swerve.setAutoAimLocation(AutoAimLocation.MR);
                    swerve.enableAutoAimController(true);
                }
                if (!buttonPressed) {
                    wasPreviouslyPressed = false;
                    swerve.enableAutoAimController(false);
                }
            } else if (p.getRawButton(3)) {
                buttonPressed = true;
                if (buttonPressed && !wasPreviouslyPressed) {
                    wasPreviouslyPressed = true;
                    swerve.setAutoAimLocation(AutoAimLocation.RL);
                    swerve.enableAutoAimController(true);
                }
                if (!buttonPressed) {
                    wasPreviouslyPressed = false;
                    swerve.enableAutoAimController(false);
                }
            } else if (p.getRawButton(2)) {
                buttonPressed = true;
                if (buttonPressed && !wasPreviouslyPressed) {
                    wasPreviouslyPressed = true;
                    swerve.setAutoAimLocation(AutoAimLocation.RM);
                    swerve.enableAutoAimController(true);
                }
                if (!buttonPressed) {
                    wasPreviouslyPressed = false;
                    swerve.enableAutoAimController(false);
                }
            } else if (p.getRawButton(1)) {
                buttonPressed = true;
                if (buttonPressed && !wasPreviouslyPressed) {
                    wasPreviouslyPressed = true;
                    swerve.setAutoAimLocation(AutoAimLocation.RR);
                    swerve.enableAutoAimController(true);
                }
                if (!buttonPressed) {
                    wasPreviouslyPressed = false;
                    swerve.enableAutoAimController(false);
                }
            } else {
                buttonPressed = false;
                wasPreviouslyPressed = false;
                swerve.enableAutoAimController(false);
            }
        }
    }
}
