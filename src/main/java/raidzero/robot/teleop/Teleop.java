package raidzero.robot.teleop;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import raidzero.robot.submodules.Arm;
import raidzero.robot.utils.JoystickUtils;
import edu.wpi.first.math.MathUtil;

public class Teleop {

    private static Teleop instance = null;
    private static XboxController p1 = new XboxController(0);
    private static XboxController p2 = new XboxController(1);

    private static final Arm arm = Arm.getInstance();
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
    private double[] target = { 0, 1.8, 0, 0 };

    private void p1Loop(XboxController p) {

        rampRate = SmartDashboard.getNumber("Ramp Rate", 0);
        SmartDashboard.putNumber("Ramp Rate", rampRate);
        SmartDashboard.putNumber("Target EE X", target[0]);
        SmartDashboard.putNumber("Target EE Y", target[1]);
        SmartDashboard.putNumber("Target Lower Angle", target[2]);
        SmartDashboard.putNumber("Target Upper Angle", target[3]);

        // arm.setArmRampRate(rampRate);

        if (p.getRightBumperPressed())
            mode = 1;
        else if (p.getBackButtonPressed())
            mode = 2;
        else if (p.getStartButtonPressed())
            mode = 3;

        if (mode == 1)
            arm.moveArm(p.getRightX() * 0.2, p.getLeftX() * 0.2);
        else if (mode == 2) {
            if (p.getYButtonPressed()) {
                arm.moveToAngle(36.7, 36.7);
            } else if (p.getXButtonPressed()) {
                arm.moveToAngle(135.7, -36.7);
            } else if (p.getAButtonPressed()) {
                arm.moveToAngle(90, 0);
            }
        } else if (mode == 3) {
            if (Math.abs(target[0]) <= 1.8 && target[1] <= 1.8 && target[1] >= -0.2) {
                target[0] += MathUtil.applyDeadband(p.getRightX() * 0.07, 0.05);
                target[1] += MathUtil.applyDeadband(p.getLeftY() * -0.07, 0.05);
            } else if (Math.abs(target[0]) > 1.8) {
                if (Math.signum(target[0]) == -1)
                    target[0] = -1.8;
                else
                    target[0] = 1.8;
            } else if (target[1] > 1.8)
                target[1] = 1.8;
            else if (target[1] < -0.2)
                target[1] = -0.2;

            // Reset Pose
            if (p.getLeftBumperPressed()) {
                target[0] = 0;
                target[1] = 1.8;
            }
            target[2] = arm.invKin(target)[0];
            target[3] = arm.invKin(target)[1];
            arm.moveToAngle(target[2], target[3]);
        }

    }

    private void p2Loop(XboxController p) {
    }

}
