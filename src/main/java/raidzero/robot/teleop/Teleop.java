package raidzero.robot.teleop;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import raidzero.robot.Constants.ArmConstants;
import raidzero.robot.submodules.Arm;
import raidzero.robot.submodules.Intake;
import raidzero.robot.submodules.Swerve;
import raidzero.robot.submodules.Swerve.AutoAimLocation;
import raidzero.robot.submodules.Wrist;
import raidzero.robot.utils.JoystickUtils;

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
        if (DriverStation.getAlliance() == Alliance.Blue) {
            blue = true;
            reverse = 1;
        } else {
            blue = false;
            reverse = -1;
        }
    }

    public void onLoop() {
        /**
         * p1 controls
         */
        p1Loop(p1);
        /**
         * p2 controls
         */
        // p2Loop(p2);
        /**
         * p3 controls
         */
        // p3Loop(p3);

    }

    private double[] target = { 0, 0.15 };

    private boolean aiming = false;
    private boolean noSafenoProblemo = false;

    private boolean justPressed = false;

    private void p1Loop(XboxController p) {
        SmartDashboard.putBoolean("Aiming", aiming);
        SmartDashboard.putBoolean("Safety", noSafenoProblemo);
        if (p.getYButtonPressed()) {
            aiming = true;
        }
        if (p.getBButtonPressed()) {
            aiming = false;
        }
        // if (p.getAButtonPressed()) {
        // noSafenoProblemo = !noSafenoProblemo;
        // }
        if (p.getXButtonPressed()) {
            if (blue)
                swerve.zeroHeading(0);
            else
                swerve.zeroHeading(180);
        }

        // if (!aiming)
            // swerve.drive(
            //         JoystickUtils.deadband(-p.getLeftY() * arm.tooFasttooFurious() * reverse),
            //         JoystickUtils.deadband(-p.getLeftX() * arm.tooFasttooFurious() * reverse),
            //         JoystickUtils.deadband(-p.getRightX() * arm.tooFasttooFurious()),
        //             true);
        // else
        //     swerve.drive(
        //             JoystickUtils.aimingDeadband(-p.getLeftY() * 0.25 * reverse),
        //             JoystickUtils.aimingDeadband(-p.getLeftX() * 0.25 * reverse),
        //             JoystickUtils.aimingDeadband(-p.getRightX() * 0.25),
        //             true);

        
        if(p.getAButton()) {
            swerve.testSplineAutoAim(true);
        }
        if(p.getLeftBumper()) {
            swerve.drive(
                JoystickUtils.aimingDeadband(-p.getLeftY() * 0.25 * reverse),
                JoystickUtils.aimingDeadband(-p.getLeftX() * 0.25 * reverse),
                JoystickUtils.aimingDeadband(-p.getRightX() * 0.25),
                true
            );
        }

        // Auto Alignments
        // if (blue && ((!arm.isGoingHome() && arm.isSafe())
        //         || noSafenoProblemo)) {
        //     if (p.getRightBumper())
        //         swerve.autoAim(AutoAimLocation.BR_LOAD);
        //     else if (p.getLeftBumper())
        //         swerve.autoAim(AutoAimLocation.BL_LOAD);

        // } else if (!blue && ((!arm.isGoingHome() && arm.isSafe())
        //         || noSafenoProblemo)) {
        //     if (p.getRightBumper())
        //         swerve.autoAim(AutoAimLocation.RR_LOAD);
        //     else if (p.getLeftBumper())
        //         swerve.autoAim(AutoAimLocation.RL_LOAD);
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

                arm.moveThreePronged(-.10, 0.7, 90, -.01, 1.4, 90,
                        -ArmConstants.HUMAN_PICKUP_STATION[0],
                        ArmConstants.HUMAN_PICKUP_STATION[1], 180);
            }
            // High Grid
            else if (p.getBButtonPressed() &&
                    !swerve.isOverLimit() &&
                    !arm.isGoingHome() &&
                    arm.isOnTarget() &&
                    arm.isSafe()) {
                arm.moveTwoPronged(
                        -ArmConstants.INTER_GRID_HIGH[0],
                        ArmConstants.INTER_GRID_HIGH[1],
                        70,
                        -ArmConstants.GRID_HIGH[0],
                        ArmConstants.GRID_HIGH[1],
                        155);
            }
            // Medium Grid
            else if (p.getAButtonPressed() &&
                    !swerve.isOverLimit() &&
                    !arm.isGoingHome() &&
                    arm.isOnTarget() &&
                    arm.isSafe()) {
                arm.moveTwoPronged(
                        -ArmConstants.INTER_GRID_MEDIUM[0],
                        ArmConstants.INTER_GRID_MEDIUM[1],
                        70,
                        -ArmConstants.GRID_MEDIUM[0],
                        ArmConstants.GRID_MEDIUM[1],
                        155);
            }
            // Floor Intake
            else if (p.getXButtonPressed() &&
                    !swerve.isOverLimit() &&
                    !arm.isGoingHome() &&
                    arm.isOnTarget() &&
                    arm.isSafe()) {
                arm.moveTwoPronged(
                        -ArmConstants.INTER_FLOOR_INTAKE[0],
                        ArmConstants.INTER_FLOOR_INTAKE[1],
                        155,
                        -ArmConstants.FLOOR_INTAKE[0],
                        ArmConstants.FLOOR_INTAKE[1],
                        155);
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

            arm.moveToPoint(target[0], target[1], 0);
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

    private void p3Loop(GenericHID p) {
        // Human Pickup Station
        if (p.getRawButtonPressed(10) &&
                ((!swerve.isOverLimit() && !arm.isGoingHome() && arm.isOnTarget() && arm.isSafe())
                        || noSafenoProblemo)) {
            arm.configSmartMotionConstraints(
                    ArmConstants.LOWER_MAX_VEL * 1.5,
                    ArmConstants.LOWER_MAX_ACCEL * 1.5,
                    ArmConstants.UPPER_MAX_VEL * 0.75,
                    ArmConstants.UPPER_MAX_ACCEL * 0.75);

            arm.moveThreePronged(
                    -ArmConstants.INTER_HUMAN_PICKUP_STATION[0],
                    ArmConstants.INTER_HUMAN_PICKUP_STATION[1],
                    90,
                    -ArmConstants.INTER2_HUMAN_PICKUP_STATION[0],
                    ArmConstants.INTER2_HUMAN_PICKUP_STATION[1],
                    90,
                    -ArmConstants.HUMAN_PICKUP_STATION[0],
                    ArmConstants.HUMAN_PICKUP_STATION[1],
                    160);
        }
        // High Grid
        else if (p.getRawButtonPressed(14) &&
                ((!swerve.isOverLimit() && !arm.isGoingHome() && arm.isOnTarget() && arm.isSafe())
                        || noSafenoProblemo)) {
            arm.moveTwoPronged(
                    -ArmConstants.INTER_GRID_HIGH[0],
                    ArmConstants.INTER_GRID_HIGH[1],
                    70,
                    -ArmConstants.GRID_HIGH[0],
                    ArmConstants.GRID_HIGH[1],
                    155);
        }
        // Medium Grid
        else if (p.getRawButtonPressed(15) &&
                ((!swerve.isOverLimit() && !arm.isGoingHome() && arm.isOnTarget() && arm.isSafe())
                        || noSafenoProblemo)) {
            arm.moveTwoPronged(
                    -ArmConstants.INTER_GRID_MEDIUM[0],
                    ArmConstants.INTER_GRID_MEDIUM[1],
                    70,
                    -ArmConstants.GRID_MEDIUM[0],
                    ArmConstants.GRID_MEDIUM[1],
                    155);
        }
        // Floor Intake
        else if (p.getRawButtonPressed(16) &&
                ((!swerve.isOverLimit() && !arm.isGoingHome() && arm.isOnTarget() && arm.isSafe())
                        || noSafenoProblemo)) {
            arm.moveThreePronged(
                    ArmConstants.INTER_FLOOR_INTAKE[0],
                    ArmConstants.INTER_FLOOR_INTAKE[1],
                    45,
                    ArmConstants.INTER2_FLOOR_INTAKE[0],
                    ArmConstants.INTER2_FLOOR_INTAKE[1],
                    90,
                    ArmConstants.FLOOR_INTAKE[0],
                    ArmConstants.FLOOR_INTAKE[1],
                    165);
        }
        // Reverse Stage
        else if (p.getRawAxis(0) == 1 &&
                ((!swerve.isOverLimit() && !arm.isGoingHome() && !arm.isSafe())
                        || noSafenoProblemo)) {
            arm.reverseStage();
        }
        // Go Home
        else if (p.getRawButtonPressed(13)) {
            // arm.configSmartMotionConstraints(
            //         ArmConstants.LOWER_MAX_VEL * 1.5,
            //         ArmConstants.LOWER_MAX_ACCEL * 1.5,
            //         ArmConstants.UPPER_MAX_VEL * 1.5,
            //         ArmConstants.UPPER_MAX_ACCEL * 1.5);
            arm.goHome();
        }

        // Intake
        if (p.getRawButton(12)) {
            intake.setPercentSpeed(0.7);
        } else if (p.getRawButton(11)) {
            intake.setPercentSpeed(-0.7);
        } else {
            intake.holdPosition();
        }

        // Auto Alignments
        // if (blue && ((!arm.isGoingHome() && arm.isSafe())
        //         || noSafenoProblemo)) {
        //     if (p.getRawButton(9)) {
        //         swerve.autoAim(AutoAimLocation.BLL);
        //     } else if (p.getRawButton(8)) {
        //         swerve.autoAim(AutoAimLocation.BLM);
        //     } else if (p.getRawButton(7)) {
        //         swerve.autoAim(AutoAimLocation.BLR);
        //     } else if (p.getRawButton(6)) {
        //         swerve.autoAim(AutoAimLocation.BML);
        //     } else if (p.getRawButton(5)) {
        //         swerve.autoAim(AutoAimLocation.BMM);
        //     } else if (p.getRawButton(4)) {
        //         swerve.autoAim(AutoAimLocation.BMR);
        //     } else if (p.getRawButton(3)) {
        //         swerve.autoAim(AutoAimLocation.BRL);
        //     } else if (p.getRawButton(2)) {
        //         swerve.autoAim(AutoAimLocation.BRM);
        //     } else if (p.getRawButton(1)) {
        //         swerve.autoAim(AutoAimLocation.BRR);
        //     } else {
        //     }
        // } else if (!blue && ((!arm.isGoingHome() && arm.isSafe())
        //         || noSafenoProblemo)) {
        //     if (p.getRawButton(9)) {
        //         swerve.autoAim(AutoAimLocation.RLL);
        //     } else if (p.getRawButton(8)) {
        //         swerve.autoAim(AutoAimLocation.RLM);
        //     } else if (p.getRawButton(7)) {
        //         swerve.autoAim(AutoAimLocation.RLR);
        //     } else if (p.getRawButton(6)) {
        //         swerve.autoAim(AutoAimLocation.RML);
        //     } else if (p.getRawButton(5)) {
        //         swerve.autoAim(AutoAimLocation.RMM);
        //     } else if (p.getRawButton(4)) {
        //         swerve.autoAim(AutoAimLocation.RMR);
        //     } else if (p.getRawButton(3)) {
        //         swerve.autoAim(AutoAimLocation.RRL);
        //     } else if (p.getRawButton(2)) {
        //         swerve.autoAim(AutoAimLocation.RRM);
        //     } else if (p.getRawButton(1)) {
        //         swerve.autoAim(AutoAimLocation.RRR);
        //     } else {
        //     }
        // }
    }
}
