package raidzero.robot.teleop;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import raidzero.robot.Constants.ArmConstants;
import raidzero.robot.submodules.Arm;
import raidzero.robot.submodules.Intake;
import raidzero.robot.submodules.Wrist;
import raidzero.robot.utils.JoystickUtils;
import raidzero.robot.wrappers.LazyCANSparkMax;

import javax.naming.directory.AttributeModificationException;

import edu.wpi.first.math.MathUtil;

public class Teleop {

    private static Teleop instance = null;
    private static XboxController p1 = new XboxController(0);
    private static XboxController p2 = new XboxController(1);

    private static final Arm arm = Arm.getInstance();
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

        rampRate = SmartDashboard.getNumber("Ramp Rate", 0);
        SmartDashboard.putNumber("Ramp Rate", rampRate);
        SmartDashboard.putNumber("Target EE X", target[0]);
        SmartDashboard.putNumber("Target EE Y", target[1]);

        // arm.setArmRampRate(rampRate);

        if (p.getRightBumperPressed())
            mode = 1; // Joystick
        else if (p.getBackButtonPressed())
            mode = 2; // Setpoint SM
        else if (p.getStartButtonPressed())
            mode = 3; // Joystick with Inv Kin.
        else if (p.getLeftBumperPressed())
            mode = 4; // Go Home

        if (mode == 1) {
            arm.moveArm(p.getLeftX() * 0.2, p.getRightX() * 0.2);
            arm.getWrist().setPercentSpeed(p.getLeftY() * 0.2);
        } else if (mode == 2) {
            if (p.getYButtonPressed()) {
                arm.configSmartMotionConstraints(
                        ArmConstants.LOWER_MAX_VEL * 1.5,
                        ArmConstants.LOWER_MAX_ACCEL * 1.5,
                        ArmConstants.UPPER_MAX_VEL * 0.75,
                        ArmConstants.UPPER_MAX_ACCEL * 0.75);

                arm.moveThreePronged(-.10, 0.7, 90, -.01, 1.4, 90, -ArmConstants.HUMAN_PICKUP_STATION[0],
                        ArmConstants.HUMAN_PICKUP_STATION[1], 170);

                // arm.moveToAngle(42.1085066795, -255.950172901);
            } else if (p.getBButtonPressed()) {
                arm.moveTwoPronged(-.05, 1.5, 0, -ArmConstants.GRID_HIGH[0], ArmConstants.GRID_HIGH[1], 180);
                // arm.moveToAngle(70, -90);
            } else if (p.getXButtonPressed()) {
                arm.moveToPoint(-ArmConstants.FLOOR_INTAKE[0], ArmConstants.FLOOR_INTAKE[1], 180);
            } else if (p.getAButtonPressed()) {
                // arm.moveToAngle(110, -270);
                arm.moveTwoPronged(-0.7, 0.7, 0, -0.5, 0.5, 0);
            }
        } else if (mode == 3) {
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

        if (Math.abs(p.getRightTriggerAxis() - p.getLeftTriggerAxis()) >= 0.2) {
            intake.setPercentSpeed(p.getRightTriggerAxis() - p.getLeftTriggerAxis());
        } else {
            intake.holdPosition();
        }

        // if (p.getRightTriggerAxis() - p.getLeftTriggerAxis() >= 0.2) {
        // arm.getWrist().getIntake().setPercentSpeed(p.getRightTriggerAxis() -
        // p.getLeftTriggerAxis());
        // } else if (Math.abs(p.getRightTriggerAxis() - p.getLeftTriggerAxis()) <= 0.2)
        // {
        // arm.getWrist().getIntake().noOpenLoop();
        // System.out.println("stop");
        // } else if (!arm.getWrist().getIntake().getOpenLoop()) {
        // arm.getWrist().getIntake().setDesiredPosition(arm.getWrist().getIntake().getFinalTarget());
        // }

    }

    private int shift = 0;

    private void p2Loop(XboxController p) {

    }
}
