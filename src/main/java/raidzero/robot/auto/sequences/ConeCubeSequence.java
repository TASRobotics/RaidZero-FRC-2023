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

public class ConeCubeSequence extends AutoSequence {
    private static final Swerve mSwerve = Swerve.getInstance();

    private PathPlannerTrajectory mOut = PathPlanner.loadPath("CC Pickup", SwerveConstants.MAX_DRIVE_VEL_MPS * 1.0,
            SwerveConstants.MAX_DRIVE_ACCEL_MPSPS * 1.0);
    private PathPlannerTrajectory mReturn = PathPlanner.loadPath("CC Score", SwerveConstants.MAX_DRIVE_VEL_MPS * 1.0,
            SwerveConstants.MAX_DRIVE_ACCEL_MPSPS * 1.0);

    private PathPlannerTrajectory mBalance = PathPlanner.loadPath("CC Balance",
            SwerveConstants.MAX_DRIVE_VEL_MPS * 1.0,
            SwerveConstants.MAX_DRIVE_ACCEL_MPSPS * 1.0);

    public ConeCubeSequence() {
        PathPlannerTrajectory.transformTrajectoryForAlliance(mOut, DriverStation.getAlliance());
        PathPlannerTrajectory.transformTrajectoryForAlliance(mReturn, DriverStation.getAlliance());
        PathPlannerTrajectory.transformTrajectoryForAlliance(mBalance, DriverStation.getAlliance());
    }

    @Override
    public void sequence() {
        addAction(
                new SeriesAction(Arrays.asList(
                        // Score Cone
                        new RunIntakeAction(0.1, 0.5),
                        new MoveTwoPronged(-ArmConstants.INTER_GRID_HIGH[0],
                                ArmConstants.INTER_GRID_HIGH[1],
                                ArmConstants.INTER_GRID_HIGH[2],
                                -ArmConstants.GRID_HIGH[0], ArmConstants.GRID_HIGH[1],
                                ArmConstants.GRID_HIGH[2]),
                        new RunIntakeAction(0.5, -1),

                        // Go To Cube + Scoop
                        new ParallelAction(Arrays.asList(
                                new AsyncArmHomeAction(),
                                new DrivePath(mOut),
                                new SeriesAction(Arrays.asList(
                                        new WaitAction(1.3),
                                        new MoveThreePronged(
                                                ArmConstants.INTER_FLOOR_INTAKE[0],
                                                ArmConstants.INTER_FLOOR_INTAKE[1],
                                                ArmConstants.INTER_FLOOR_INTAKE[2],
                                                ArmConstants.INTER2_FLOOR_INTAKE[0],
                                                ArmConstants.INTER2_FLOOR_INTAKE[1],
                                                ArmConstants.INTER2_FLOOR_INTAKE[2],
                                                ArmConstants.FLOOR_INTAKE[0],
                                                ArmConstants.FLOOR_INTAKE[1],
                                                ArmConstants.FLOOR_INTAKE[2]))),
                                new RunIntakeAction(2, -0.7))),

                        // Return to community
                        new ParallelAction(Arrays.asList(
                                new ArmHomeAction(),
                                new DrivePath(mReturn),
                                new RunIntakeAction(3, -0.7))),

                        // Score Cube
                        new MoveTwoPronged(-ArmConstants.INTER_CUBE_GRID_HIGH[0],
                                ArmConstants.INTER_CUBE_GRID_HIGH[1],
                                ArmConstants.INTER_CUBE_GRID_HIGH[2],
                                -ArmConstants.CUBE_GRID_HIGH[0], ArmConstants.CUBE_GRID_HIGH[1],
                                ArmConstants.CUBE_GRID_HIGH[2]),

                        new RunIntakeAction(0.5, 0.5),

                        new ParallelAction(Arrays.asList(
                                new ArmHomeAction(),
                                new DrivePath(mBalance))),
                        new LambdaAction(() -> mSwerve.rotorBrake(true))

                )));
    }

    @Override
    public void onEnded() {
    }

    @Override
    public String getName() {
        return "Cone Cube Sequence";
    }
}
