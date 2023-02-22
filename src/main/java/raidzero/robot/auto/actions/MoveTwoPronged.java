package raidzero.robot.auto.actions;

import raidzero.robot.Constants.ArmConstants;
import raidzero.robot.submodules.Arm;
import raidzero.robot.utils.TimerBoolean;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose2d;

public class MoveTwoPronged implements Action {

    private static final Arm arm = Arm.getInstance();
    // Intermediate State Constants
    private double[] xWaypointPositions = { 0, 0, 0 };
    private double[] yWaypointPositions = { 0, 0, 0 };
    private double[] wristWaypointPositions = { 0, 0, 0 };

    public MoveTwoPronged(double inter_x, double inter_y, double inter_wrist,
            double target_x, double target_y, double target_wrist) {
        xWaypointPositions = new double[2];
        yWaypointPositions = new double[2];
        wristWaypointPositions = new double[2];
        xWaypointPositions[0] = inter_x;
        xWaypointPositions[1] = target_x;
        yWaypointPositions[0] = inter_y;
        yWaypointPositions[1] = target_y;
        wristWaypointPositions[0] = inter_wrist;
        wristWaypointPositions[1] = target_wrist;
    }

    @Override
    public boolean isFinished() {
        Pose2d endPose = arm.getState()[1];
        return Math.abs(endPose.getX() - xWaypointPositions[1]) < 0.1
                && Math.abs(endPose.getY() - yWaypointPositions[1]) < 0.1;
    }

    @Override
    public void start() {
        arm.moveTwoPronged(xWaypointPositions[0], yWaypointPositions[0], wristWaypointPositions[0],
                xWaypointPositions[1], yWaypointPositions[1], wristWaypointPositions[1]);
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' started!");
    }

    @Override
    public void update() {
        // arm.update(Timer.getFPGATimestamp());
    }

    @Override
    public void done() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' finished!");
    }
}