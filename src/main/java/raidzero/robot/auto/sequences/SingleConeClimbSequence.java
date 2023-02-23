package raidzero.robot.auto.sequences;

import java.util.Arrays;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import raidzero.robot.Constants.ArmConstants;
import raidzero.robot.Constants.SwerveConstants;
import raidzero.robot.auto.actions.ArmHomeAction;
import raidzero.robot.auto.actions.DrivePath;
import raidzero.robot.auto.actions.LambdaAction;
import raidzero.robot.auto.actions.MoveTwoPronged;
import raidzero.robot.auto.actions.RunIntakeAction;
import raidzero.robot.auto.actions.SeriesAction;
import raidzero.robot.auto.actions.WaitAction;
import raidzero.robot.submodules.Intake;

public class SingleConeClimbSequence extends AutoSequence {
    private final static Intake mIntake = Intake.getInstance();

    private PathPlannerTrajectory mTrajectory = PathPlanner.loadPath("SCC Balance",
            SwerveConstants.MAX_DRIVE_VEL_MPS * 0.75,
            SwerveConstants.MAX_DRIVE_ACCEL_MPSPS, false);
    private PathPlannerTrajectory mOverRamp = PathPlanner.loadPath("TCC Over", SwerveConstants.MAX_DRIVE_VEL_MPS,
            SwerveConstants.MAX_DRIVE_ACCEL_MPSPS);
    private PathPlannerTrajectory mBalance = PathPlanner.loadPath("TCC Balance", SwerveConstants.MAX_DRIVE_VEL_MPS,
            SwerveConstants.MAX_DRIVE_ACCEL_MPSPS);

    public SingleConeClimbSequence() {
    }

    @Override
    public void sequence() {
        addAction(
                new SeriesAction(Arrays.asList(
                        new RunIntakeAction(0.2, 0.5),
                        new MoveTwoPronged(-.05, 1.5, 0, -ArmConstants.GRID_HIGH[0], ArmConstants.GRID_HIGH[1], 180),
                        new RunIntakeAction(1, -1),
                        new ArmHomeAction(),
                        new DrivePath(mOverRamp),
                        new WaitAction(1),
                        new DrivePath(mBalance))));
    }

    @Override
    public void onEnded() {

    }

    @Override
    public String getName() {
        return "Single Cone Climb Sequence";
    }
}
