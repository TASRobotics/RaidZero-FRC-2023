package raidzero.robot.auto.actions;

import raidzero.robot.Constants.ArmConstants;
import raidzero.robot.submodules.Arm;
import raidzero.robot.utils.TimerBoolean;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;

public class MoveThreePronged implements Action {

    private static final Arm arm = Arm.getInstance();
    // Multi-Staged Movement Constants
    private int stage = 0;
    // Intermediate State Constants
    private double[] xWaypointPositions = { 0, 0, 0 };
    private double[] yWaypointPositions = { 0, 0, 0 };
    private double[] wristWaypointPositions = { 0, 0, 0 };

    public MoveThreePronged(double inter_x, double inter_y, double inter_wrist,
            double inter_x2, double inter_y2, double inter_wrist2,
            double target_x, double target_y, double target_wrist) {
        xWaypointPositions = new double[3];
        yWaypointPositions = new double[3];
        wristWaypointPositions = new double[3];
        xWaypointPositions[0] = inter_x;
        xWaypointPositions[1] = inter_x2;
        xWaypointPositions[2] = target_x;

        yWaypointPositions[0] = inter_y;
        yWaypointPositions[1] = inter_y2;
        yWaypointPositions[2] = target_y;

        wristWaypointPositions[0] = inter_wrist;
        wristWaypointPositions[1] = inter_wrist2;
        wristWaypointPositions[2] = target_wrist;
    }

    @Override
    public boolean isFinished() {
        Pose2d endPose = arm.getState()[1];
        return Math.abs(endPose.getX()-xWaypointPositions[2]) < 0.1 && Math.abs(endPose.getY() - yWaypointPositions[2]) < 0.1;
    }

    @Override
    public void start() {
        stage = 0;
        arm.moveThreePronged(xWaypointPositions[0], yWaypointPositions[0], wristWaypointPositions[0],
                xWaypointPositions[1], yWaypointPositions[1], wristWaypointPositions[1], xWaypointPositions[2],
                yWaypointPositions[2], wristWaypointPositions[2]);
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' started!");
    }

    @Override
    public void update() {
        arm.update(Timer.getFPGATimestamp());
    }

    @Override
    public void done() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' finished!");
        arm.stop();
    }
}