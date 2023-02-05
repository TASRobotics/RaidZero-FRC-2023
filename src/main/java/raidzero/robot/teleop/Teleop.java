package raidzero.robot.teleop;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import raidzero.robot.submodules.Swerve;
import raidzero.robot.Constants.SwerveConstants;
import raidzero.robot.utils.JoystickUtils;

public class Teleop {

    private static Teleop instance = null;
    private static XboxController p1 = new XboxController(0);
    private static XboxController p2 = new XboxController(1);

    private static final Swerve swerve = Swerve.getInstance();
    private double rampRate = 0.0;

    public static Teleop getInstance() {
        if (instance == null) {
            instance = new Teleop();
        }
        return instance;
    }

    public void onStart() {
        swerve.zero();
    }

    /**
     * Continuously loops in teleop.
     */
    public void onLoop() {
        /**
         * shared controls
         */

        /**
         * p1 controls
         */
        p1Loop(p1);
        /**
         * p2 controls
         */
        p2Loop(p2);
    }

    private void p1Loop(XboxController p) {
        /**
         * Drive
        */

        swerve.drive(
            JoystickUtils.deadband(-p.getLeftY()) * SwerveConstants.MAX_SPEED_MPS,
            JoystickUtils.deadband(-p.getLeftX()) * SwerveConstants.MAX_SPEED_MPS,
            JoystickUtils.deadband(-p.getRightX() * SwerveConstants.MAX_SPEED_MPS),
            true
        );

    }

    private int mode = 0;
    private void p2Loop(XboxController p) {}

}

