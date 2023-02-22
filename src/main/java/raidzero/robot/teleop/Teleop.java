package raidzero.robot.teleop;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import raidzero.robot.Constants.ArmConstants;
import raidzero.robot.submodules.Arm;
import raidzero.robot.submodules.Intake;
import raidzero.robot.submodules.Wrist;
import raidzero.robot.submodules.Swerve;
import raidzero.robot.utils.JoystickUtils;

import edu.wpi.first.math.MathUtil;

public class Teleop {

    private static Teleop instance = null;
    private static XboxController p1 = new XboxController(0);
    private static XboxController p2 = new XboxController(1);
    private static GenericHID p3 = new GenericHID(2);

    private static final Arm arm = Arm.getInstance();
    private static final Swerve swerve = Swerve.getInstance();
    private static final Wrist wrist = Wrist.getInstance();
    private static final Intake intake = Intake.getInstance();

    private double rampRate = 0.0;
    private boolean delivering = false;

    public static Teleop getInstance() {
        if (instance == null) {
            instance = new Teleop();
        }
        return instance;
    }

    public void onStart() {
    }

    public void onLoop() {
        /**
         * p1 controls
         */
        p1Loop(p1);
        /**
         * p2 controls
         */
        p2Loop(p2);
        /**
         * p3 controls
         */
        p3Loop(p3);
    }

    private double[] target = { 0, 0.15 };
    private boolean aiming = false;

    private void p1Loop(XboxController p) {
        if (p.getYButtonPressed()) {
            aiming = !aiming;
        }
        if (!aiming)
            swerve.drive(
                    JoystickUtils.deadband(-p.getLeftY() * arm.tooFasttooFurious()),
                    JoystickUtils.deadband(-p.getLeftX() * arm.tooFasttooFurious()),
                    JoystickUtils.deadband(-p.getRightX() * arm.tooFasttooFurious()),
                    true);
        else
            swerve.drive(
                    JoystickUtils.aimingDeadband(-p.getLeftY() * 0.25),
                    JoystickUtils.aimingDeadband(-p.getLeftX() * 0.25),
                    JoystickUtils.aimingDeadband(-p.getRightX() * 0.25),
                    true);
    }

    private int mode = 0;

    private void p2Loop(XboxController p) {
        // rampRate = SmartDashboard.getNumber("Ramp Rate", 0);
        // SmartDashboard.putNumber("Ramp Rate", rampRate);
        // SmartDashboard.putNumber("Target EE X", target[0]);
        // SmartDashboard.putNumber("Target EE Y", target[1]);
        // SmartDashboard.putNumber("Mode", mode);

        // // arm.setArmRampRate(rampRate);

        // if (p.getRightBumperPressed())
        // mode = 1; // Joystick
        // else if (p.getBackButtonPressed())
        // mode = 2; // Setpoints
        // else if (p.getStartButtonPressed())
        // mode = 3; // Joystick with Inv Kin.
        // else if (p.getLeftBumperPressed())
        // mode = 4; // Go Home

        // if (mode == 1) {
        // arm.moveArm(p.getLeftX() * 0.2, p.getRightX() * 0.2);
        // arm.getWrist().setPercentSpeed(p.getLeftY() * 0.2);
        // } else if (mode == 2) {
        // // Human Pickup Station
        // if (p.getYButtonPressed() && !swerve.isOverLimit() && !arm.isGoingHome() &&
        // !delivering) {
        // delivering = true;
        // arm.configSmartMotionConstraints(
        // ArmConstants.LOWER_MAX_VEL * 1.5,
        // ArmConstants.LOWER_MAX_ACCEL * 1.5,
        // ArmConstants.UPPER_MAX_VEL * 0.75,
        // ArmConstants.UPPER_MAX_ACCEL * 0.75);

        // arm.moveThreePronged(-.10, 0.7, 90, -.01, 1.4, 90,
        // -ArmConstants.HUMAN_PICKUP_STATION[0],
        // ArmConstants.HUMAN_PICKUP_STATION[1], 180);
        // if (p.getYButtonPressed()) {
        // arm.reverseState();
        // }
        // }
        // // High Grid
        // else if (p.getBButtonPressed() && !swerve.isOverLimit() && !arm.isGoingHome()
        // && !delivering) {
        // delivering = true;
        // arm.moveTwoPronged(-.05, 1.5, 0, -ArmConstants.GRID_HIGH[0],
        // ArmConstants.GRID_HIGH[1], 180);
        // if (p.getBButtonPressed()) {
        // arm.reverseState();
        // }
        // }
        // // Medium Grid
        // else if (p.getAButtonPressed() && !swerve.isOverLimit() && !arm.isGoingHome()
        // && !delivering) {
        // delivering = true;
        // arm.moveTwoPronged(-0.05, 0.8, 0, -ArmConstants.GRID_MEDIUM[0],
        // ArmConstants.GRID_MEDIUM[1], 180);
        // if (p.getAButtonPressed()) {
        // arm.reverseState();
        // }
        // }
        // // Floor Intake
        // else if (p.getXButtonPressed() && !swerve.isOverLimit() &&
        // !arm.isGoingHome()) {
        // delivering = false;
        // arm.moveToPoint(-ArmConstants.FLOOR_INTAKE[0], ArmConstants.FLOOR_INTAKE[1],
        // 180);
        // }

        // } else if (mode == 3) {
        // // Extension Limits
        // if (Math.abs(target[0]) <= ArmConstants.X_EXTENSION_LIMIT && target[1] <=
        // ArmConstants.Y_EXTENSION_LIMIT
        // && target[1] >= 0) {
        // target[0] = arm.getState()[1].getX() + MathUtil.applyDeadband(p.getLeftX() *
        // 0.25, 0.05);
        // target[1] = arm.getState()[1].getY() + MathUtil.applyDeadband(p.getRightY() *
        // -0.25, 0.05);
        // }
        // // Soft Joystick Limits
        // else if (Math.abs(target[0]) > ArmConstants.X_EXTENSION_LIMIT) {
        // if (Math.signum(target[0]) == -1)
        // target[0] = -ArmConstants.X_EXTENSION_LIMIT;
        // else
        // target[0] = ArmConstants.X_EXTENSION_LIMIT;
        // } else if (target[1] > ArmConstants.Y_EXTENSION_LIMIT)
        // target[1] = ArmConstants.Y_EXTENSION_LIMIT;
        // else if (target[1] < 0)
        // target[1] = 0;

        // arm.moveToPoint(target[0], target[1], 0);
        // } else if (mode == 4) {
        // delivering = false;
        // arm.goHome();
        // mode = 0;
        // }

        // // Intake
        // if (Math.abs(p.getRightTriggerAxis() - p.getLeftTriggerAxis()) >= 0.2) {
        // intake.setPercentSpeed(p.getRightTriggerAxis() - p.getLeftTriggerAxis());
        // } else {
        // intake.holdPosition();
        // }
    }

    private void p3Loop(GenericHID p) {
        // Human Pickup Station
        if (p.getRawButtonPressed(10) && !swerve.isOverLimit() && !arm.isGoingHome() && !delivering) {
            delivering = true;
            arm.configSmartMotionConstraints(
                    ArmConstants.LOWER_MAX_VEL * 1.5,
                    ArmConstants.LOWER_MAX_ACCEL * 1.5,
                    ArmConstants.UPPER_MAX_VEL * 0.75,
                    ArmConstants.UPPER_MAX_ACCEL * 0.75);

            arm.moveThreePronged(-.10, 0.7, 90, -.01, 1.4, 90, -ArmConstants.HUMAN_PICKUP_STATION[0],
                    ArmConstants.HUMAN_PICKUP_STATION[1], 160);
        } else if (p.getRawButtonPressed(10) && delivering) {
            arm.reverseState();
        }
        // High Grid
        else if (p.getRawButtonPressed(14) && !swerve.isOverLimit() && !arm.isGoingHome() && !delivering) {
            delivering = true;
            arm.moveTwoPronged(-ArmConstants.INTER_GRID_HIGH[0], ArmConstants.INTER_GRID_HIGH[1], 70,
                    -ArmConstants.GRID_HIGH[0], ArmConstants.GRID_HIGH[1], 155);
        } else if (p.getRawButtonPressed(14) && delivering) {
            arm.reverseState();
        }
        // Medium Grid
        else if (p.getRawButtonPressed(15) && !swerve.isOverLimit() && !arm.isGoingHome() && !delivering) {
            delivering = true;
            arm.moveTwoPronged(-ArmConstants.INTER_GRID_MEDIUM[0], ArmConstants.INTER_GRID_MEDIUM[1], 70,
                    -ArmConstants.GRID_MEDIUM[0], ArmConstants.GRID_MEDIUM[1], 155);
        } else if (p.getRawButtonPressed(15) && delivering) {
            arm.reverseState();
        }
        // Floor Intake
        else if (p.getRawButtonPressed(16) && !swerve.isOverLimit() && !arm.isGoingHome()) {
            delivering = false;
            arm.moveToPoint(-ArmConstants.FLOOR_INTAKE[0], ArmConstants.FLOOR_INTAKE[1], 155);
        } else if (p.getRawButtonPressed(13)) {
            delivering = false;
            arm.goHome();
        }

        // Intake
        if (p.getRawButton(12)) {
            intake.setPercentSpeed(0.5);
        } else if (p.getRawButton(11)) {
            intake.setPercentSpeed(-0.7);
        }  else {
            intake.holdPosition();
        }

    }
}
