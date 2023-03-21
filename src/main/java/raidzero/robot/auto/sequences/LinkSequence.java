package raidzero.robot.auto.sequences;

import java.util.Arrays;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import raidzero.robot.Constants.ArmConstants;
import raidzero.robot.Constants.SwerveConstants;
import raidzero.robot.auto.actions.ArmHomeAction;
import raidzero.robot.auto.actions.AsyncArmHomeAction;
import raidzero.robot.auto.actions.DrivePath;
import raidzero.robot.auto.actions.LambdaAction;
import raidzero.robot.auto.actions.MoveThreePronged;
import raidzero.robot.auto.actions.MoveTwoPronged;
import raidzero.robot.auto.actions.ParallelAction;
import raidzero.robot.auto.actions.RunIntakeAction;
import raidzero.robot.auto.actions.SeriesAction;
import raidzero.robot.auto.actions.WaitAction;
import raidzero.robot.submodules.Swerve;

public class LinkSequence extends AutoSequence {
    private static final Swerve mSwerve = Swerve.getInstance();

    private PathPlannerTrajectory mFirstPickup = PathPlanner.loadPath("Link First Pickup",
            SwerveConstants.MAX_DRIVE_VEL_MPS * 0.75,
            SwerveConstants.MAX_DRIVE_ACCEL_MPSPS * 0.75);

    private PathPlannerTrajectory mFirstScore = PathPlanner.loadPath("Link First Score",
            SwerveConstants.MAX_DRIVE_VEL_MPS * 1.6,
            SwerveConstants.MAX_DRIVE_ACCEL_MPSPS * 1.6);

    private PathPlannerTrajectory mSecondPickup = PathPlanner.loadPath("Link Second Pickup",
            SwerveConstants.MAX_DRIVE_VEL_MPS * 0.9,
            SwerveConstants.MAX_DRIVE_ACCEL_MPSPS * 0.9);

    private PathPlannerTrajectory mSecondScore = PathPlanner.loadPath("Link Second Score",
            SwerveConstants.MAX_DRIVE_VEL_MPS * 1.6,
            SwerveConstants.MAX_DRIVE_ACCEL_MPSPS * 1.6);

    private PathPlannerTrajectory mBalance = PathPlanner.loadPath("Link Balance",
            SwerveConstants.MAX_DRIVE_VEL_MPS * 1.6,
            SwerveConstants.MAX_DRIVE_ACCEL_MPSPS * 1.6);

    public LinkSequence() {
        PathPlannerTrajectory.transformTrajectoryForAlliance(mFirstPickup, DriverStation.getAlliance());
        PathPlannerTrajectory.transformTrajectoryForAlliance(mFirstScore, DriverStation.getAlliance());
        PathPlannerTrajectory.transformTrajectoryForAlliance(mSecondPickup, DriverStation.getAlliance());
        PathPlannerTrajectory.transformTrajectoryForAlliance(mSecondScore, DriverStation.getAlliance());
        PathPlannerTrajectory.transformTrajectoryForAlliance(mBalance, DriverStation.getAlliance());
    }

    @Override
    public void sequence() {
        // addAction(
        //         new SeriesAction(Arrays.asList(
        //                 // Score Cone
        //                 new RunIntakeAction(0.1, 0.5),
        //                 new MoveTwoPronged(ArmConstants.INTER_REV_GRID_HIGH,
        //                         ArmConstants.REV_GRID_HIGH, false),
        //                 new RunIntakeAction(0.2, -1),

        //                 // Go To Cube + Scoop
        //                 new ParallelAction(Arrays.asList(
        //                         new AsyncArmHomeAction(),
        //                         new DrivePath(mFirstPickup),
        //                         new SeriesAction(Arrays.asList(
        //                                 new WaitAction(1.3),
        //                                 new MoveTwoPronged(
        //                                         ArmConstants.INTER_REV_CUBE_FLOOR_INTAKE,
        //                                         ArmConstants.REV_CUBE_FLOOR_INTAKE, false))),
        //                         new RunIntakeAction(2.5, -0.7))),

        //                 // Return to community 
        //                 new ParallelAction(Arrays.asList(
        //                         new AsyncArmHomeAction(),
        //                         new DrivePath(mFirstScore),
        //                         new SeriesAction(Arrays.asList(
        //                                 new WaitForEventMarkerAction(mFirstScore, "cScore", mSwerve.getPathingTime()),
        //                                 new MoveTwoPronged(ArmConstants.INTER_CUBE_GRID_HIGH,
        //                                         ArmConstants.CUBE_GRID_HIGH, true))),
        //                         new RunIntakeAction(2.0, -0.1))),

        //                 // Score Cube
        //                 new RunIntakeAction(0.2, 0.5),

        //                 // Go To Second Cone + Scoop
        //                 new ParallelAction(Arrays.asList(
        //                         new AsyncArmHomeAction(),
        //                         new DrivePath(mSecondPickup),
        //                         new SeriesAction(Arrays.asList(
        //                                 new WaitForEventMarkerAction(mSecondPickup, "fIntake",
        //                                         mSwerve.getPathingTime()),
        //                                 new MoveTwoPronged(
        //                                         ArmConstants.INTER_REV_FLIPPED_CONE_FLOOR_INTAKE,
        //                                         ArmConstants.REV_FLIPPED_CONE_FLOOR_INTAKE, false))),
        //                         new RunIntakeAction(2.5, 1.0))),

        //                 // Return to community
        //                 new ParallelAction(Arrays.asList(
        //                         new AsyncArmHomeAction(),
        //                         new DrivePath(mSecondScore),
        //                         new SeriesAction(Arrays.asList(
        //                                 new WaitForEventMarkerAction(mSecondScore, "cScore",
        //                                         mSwerve.getPathingTime()),
        //                                 new MoveTwoPronged(ArmConstants.INTER_GRID_HIGH,
        //                                         ArmConstants.GRID_HIGH, true))),
        //                         new RunIntakeAction(1.5, 0.35))),

        //                 // Score Cone
        //                 new RunIntakeAction(0.2, -1),

                        new ParallelAction(Arrays.asList(
                                new ArmHomeAction(),
                                new SeriesAction(Arrays.asList(
                                    new DrivePath(mBalance),
                                    new AutoBalanceAction(false, 20))
                                ))),
                        new LambdaAction(() -> mSwerve.rotorBrake(true))

        //         )));
    }

    @Override
    public void onEnded() {
    }

    @Override
    public String getName() {
        return "Link Sequence";
    }
}
