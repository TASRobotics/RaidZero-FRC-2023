package raidzero.robot.teleop;

import edu.wpi.first.wpilibj.XboxController;
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

    private static final Arm arm = Arm.getInstance();
    private static final Swerve swerve = Swerve.getInstance();
    private static final Wrist wrist = Wrist.getInstance();
    private static final Intake intake = Intake.getInstance();

    private double rampRate = 0.0;

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
    }

    private int mode = 0;
    private double[] target = { 0, 0.15 };

    private void p1Loop(XboxController p) {
        /**
         * Drive
        */
        boolean turning = p.getRawButton(12);
        //System.out.println(p.getLeftY());

        wrist.setPercentSpeed(p.getRightY()*.1);
        rampRate = SmartDashboard.getNumber("Ramp Rate", 0);
        SmartDashboard.putNumber("Ramp Rate", rampRate);
        swerve.setThrottleRampRate(rampRate);

        swerve.drive(
                JoystickUtils.deadband(-p.getLeftY() * arm.tooFasttooFurious()),
                JoystickUtils.deadband(-p.getLeftX() * arm.tooFasttooFurious()),
                JoystickUtils.deadband(-p.getRightX() * arm.tooFasttooFurious()),
                true);

        // if(p.getYButton()) {
        // swerve.testModule(1, 0.25, 0.25);
        // } else if(p.getAButton()) {
        // swerve.testModule(2, 0.25, 0.25);
        // } else if(p.getXButton()) {
        // swerve.testModule(3, 0.25, 0.25);
        // } else if(p.getBButton()) {
        // swerve.testModule(4, 0.25, 0.25);
        // } else {
        // swerve.stop();
        // }
    }

    private void p2Loop(XboxController p) {
        rampRate = SmartDashboard.getNumber("Ramp Rate", 0);
        SmartDashboard.putNumber("Ramp Rate", rampRate);
        SmartDashboard.putNumber("Target EE X", target[0]);
        SmartDashboard.putNumber("Target EE Y", target[1]);

        // arm.setArmRampRate(rampRate);

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
            arm.getWrist().setPercentSpeed(p.getLeftY() * 0.2);
        } else if (mode == 2) {
            // Human Pckup Station
            if (p.getYButtonPressed()) {
                arm.configSmartMotionConstraints(
                        ArmConstants.LOWER_MAX_VEL * 1.5,
                        ArmConstants.LOWER_MAX_ACCEL * 1.5,
                        ArmConstants.UPPER_MAX_VEL * 0.75,
                        ArmConstants.UPPER_MAX_ACCEL * 0.75);

                arm.moveThreePronged(-.10, 0.7, 90, -.01, 1.4, 90, -ArmConstants.HUMAN_PICKUP_STATION[0],
                        ArmConstants.HUMAN_PICKUP_STATION[1], 170);
            }
            // High Grid
            else if (p.getBButtonPressed()) {
                arm.moveTwoPronged(-.05, 1.5, 0, -ArmConstants.GRID_HIGH[0], ArmConstants.GRID_HIGH[1], 180);
            }
            // Medium Grid
            else if (p.getAButtonPressed()) {
                // arm.moveToAngle(110, -270);
                arm.moveTwoPronged(-0.05, 0.8, 0, -ArmConstants.GRID_MEDIUM[0], ArmConstants.GRID_MEDIUM[1], 180);
            }
            // Floor Intake
            else if (p.getXButtonPressed()) {
                arm.moveToPoint(-ArmConstants.FLOOR_INTAKE[0], ArmConstants.FLOOR_INTAKE[1], 180);
            }

        } else if (mode == 3) {
            // Extension Limits
            if (Math.abs(target[0]) <= ArmConstants.X_EXTENSION_LIMIT && target[1] <= ArmConstants.Y_EXTENSION_LIMIT
                    && target[1] >= 0) {
                target[0] = arm.getState()[1].getX() + MathUtil.applyDeadband(p.getLeftX() * 0.25, 0.05);
                target[1] = arm.getState()[1].getY() + MathUtil.applyDeadband(p.getRightY() * -0.25, 0.05);
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

        // Intake
        if (Math.abs(p.getRightTriggerAxis() - p.getLeftTriggerAxis()) >= 0.2) {
            intake.setPercentSpeed(p.getRightTriggerAxis() - p.getLeftTriggerAxis());
        } else {
            intake.holdPosition();
        }

    }

}

