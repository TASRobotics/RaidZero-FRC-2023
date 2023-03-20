package raidzero.robot.auto.sequences;

import java.util.Arrays;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation;

import raidzero.robot.Constants.ArmConstants;
import raidzero.robot.Constants.SwerveConstants;
import raidzero.robot.auto.actions.ArmHomeAction;
import raidzero.robot.auto.actions.AsyncArmHomeAction;
import raidzero.robot.auto.actions.AutoBalanceAction;
import raidzero.robot.auto.actions.DrivePath;
import raidzero.robot.auto.actions.LambdaAction;
import raidzero.robot.auto.actions.MoveTwoPronged;
import raidzero.robot.auto.actions.ParallelAction;
import raidzero.robot.auto.actions.RunIntakeAction;
import raidzero.robot.auto.actions.SeriesAction;
import raidzero.robot.auto.actions.WaitForEventMarkerAction;
import raidzero.robot.submodules.Swerve;

public class SingleConeClimbSequence extends AutoSequence {
    private static final Swerve mSwerve = Swerve.getInstance();

    private PathPlannerTrajectory mOverRamp = PathPlanner.loadPath("SCC Over", SwerveConstants.MAX_DRIVE_VEL_MPS * 0.5,
            SwerveConstants.MAX_DRIVE_ACCEL_MPSPS * 0.5);
    private PathPlannerTrajectory mBalance = PathPlanner.loadPath("SCC Balance",
            SwerveConstants.MAX_DRIVE_VEL_MPS * 0.5,
            SwerveConstants.MAX_DRIVE_ACCEL_MPSPS * 0.5);

    public SingleConeClimbSequence() {
        PathPlannerTrajectory.transformTrajectoryForAlliance(mOverRamp, DriverStation.getAlliance());
        PathPlannerTrajectory.transformTrajectoryForAlliance(mBalance, DriverStation.getAlliance());
    }

    @Override
    public void sequence() {
        addAction(
                new SeriesAction(Arrays.asList(
                        new RunIntakeAction(0.1, 0.5),
                        new MoveTwoPronged(ArmConstants.INTER_GRID_HIGH,
                                ArmConstants.GRID_HIGH, true),
                        new RunIntakeAction(0.5, -1),

                        // Get Cube
                        new ParallelAction(Arrays.asList(
                                new AsyncArmHomeAction(),
                                new DrivePath(mOverRamp),
                                new SeriesAction(Arrays.asList(
                                        new WaitForEventMarkerAction(mOverRamp, "fIntake",
                                                mSwerve.getPathingTime()),
                                        new MoveTwoPronged(
                                                ArmConstants.INTER_REV_CUBE_FLOOR_INTAKE,
                                                ArmConstants.REV_CUBE_FLOOR_INTAKE, false))),
                                new RunIntakeAction(3.0, -0.7))),

                        // Balance
                        new ParallelAction(Arrays.asList(
                                new AsyncArmHomeAction(),
                                new SeriesAction(Arrays.asList(
                                        new DrivePath(mBalance),
                                        new AutoBalanceAction(false))),
                                new RunIntakeAction(3.0, -0.7))),
                        new LambdaAction(() -> mSwerve.rotorBrake(true)))));
    }

    @Override
    public void onEnded() {
    }

    @Override
    public String getName() {
        return "Single Cone Climb Sequence";
    }
}
