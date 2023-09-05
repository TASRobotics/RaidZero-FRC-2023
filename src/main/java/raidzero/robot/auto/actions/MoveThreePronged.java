package raidzero.robot.auto.actions;

import raidzero.robot.Constants.ArmConstants;
import raidzero.robot.submodules.Arm;
import raidzero.robot.utils.ArmPurePursuit;
import raidzero.robot.utils.TimerBoolean;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

public class MoveThreePronged implements Action {

    private static final ArmPurePursuit armPurePursuit = ArmPurePursuit.getInstance();
    private static final Arm arm = Arm.getInstance();

    // Intermediate State Constants
    private double[] xWaypointPositions = { 0, 0, 0 };
    private double[] yWaypointPositions = { 0, 0, 0 };
    private double[] wristWaypointPositions = { 0, 0, 0 };
    private Translation2d[] cartWaypointPositions;

    public static boolean usePurePursuit = true;

    public MoveThreePronged(double[] inter, double[] inter2, double[] target, boolean front) {
        double reverse = front ? -1:1;
        xWaypointPositions = new double[3];
        yWaypointPositions = new double[3];
        wristWaypointPositions = new double[3];
        xWaypointPositions[0] = reverse*inter[0];
        xWaypointPositions[1] = reverse*inter2[0];
        xWaypointPositions[2] = reverse*target[0];

        yWaypointPositions[0] = inter[1];
        yWaypointPositions[1] = inter2[1];
        yWaypointPositions[2] = target[1];

        wristWaypointPositions[0] = inter[2];
        wristWaypointPositions[1] = inter2[2];
        wristWaypointPositions[2] = target[2];

        cartWaypointPositions = new Translation2d[]{
            new Translation2d(xWaypointPositions[0], yWaypointPositions[0]),
            new Translation2d(xWaypointPositions[1], yWaypointPositions[1]),
            new Translation2d(xWaypointPositions[2], yWaypointPositions[2])};
    }

    @Override
    public boolean isFinished() {
        Pose2d endPose = arm.getState()[1];
        return Math.abs(endPose.getX() - xWaypointPositions[2]) < 0.1
                && Math.abs(endPose.getY() - yWaypointPositions[2]) < 0.1;
    }

    @Override
    public void start() {
        if(usePurePursuit){
            armPurePursuit.makeWayPoints(cartWaypointPositions);
        }
        arm.moveThreePronged(new double[] { xWaypointPositions[0], yWaypointPositions[0], wristWaypointPositions[0] },
                new double[] { xWaypointPositions[1], yWaypointPositions[1], wristWaypointPositions[1] },
                new double[] { xWaypointPositions[2], yWaypointPositions[2], wristWaypointPositions[2] });
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' started!");
    }

    @Override
    public void update() {
        if(usePurePursuit){
            armPurePursuit.updateSpeeds();
        }
        // arm.update(Timer.getFPGATimestamp());
    }

    @Override
    public void done() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' finished!");
    }
}