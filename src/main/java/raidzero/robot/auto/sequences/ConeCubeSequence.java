package raidzero.robot.auto.sequences;

import java.util.Arrays;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation;

import raidzero.robot.Constants.ArmConstants;
import raidzero.robot.Constants.SwerveConstants;
import raidzero.robot.auto.actions.ArmHomeAction;
import raidzero.robot.auto.actions.DrivePath;
import raidzero.robot.auto.actions.LambdaAction;
import raidzero.robot.auto.actions.MoveThreePronged;
import raidzero.robot.auto.actions.MoveTwoPronged;
import raidzero.robot.auto.actions.RunIntakeAction;
import raidzero.robot.auto.actions.SeriesAction;
import raidzero.robot.submodules.Swerve;

public class ConeCubeSequence extends AutoSequence {
    private static final Swerve mSwerve = Swerve.getInstance();

    private PathPlannerTrajectory mOut = PathPlanner.loadPath("CC Out", SwerveConstants.MAX_DRIVE_VEL_MPS * 0.5,
            SwerveConstants.MAX_DRIVE_ACCEL_MPSPS * 0.5);
    private PathPlannerTrajectory mReturn = PathPlanner.loadPath("CC Score", SwerveConstants.MAX_DRIVE_VEL_MPS * 0.5,
            SwerveConstants.MAX_DRIVE_ACCEL_MPSPS * 0.5);

    public ConeCubeSequence() {
        PathPlannerTrajectory.transformTrajectoryForAlliance(mOut, DriverStation.getAlliance());
        PathPlannerTrajectory.transformTrajectoryForAlliance(mReturn, DriverStation.getAlliance());
    }

    @Override
    public void sequence() {
        addAction(
                new SeriesAction(Arrays.asList(
                        new RunIntakeAction(0.1, 0.5),
                        new MoveTwoPronged(-ArmConstants.INTER_GRID_HIGH[0], ArmConstants.INTER_GRID_HIGH[1], 70,
                                -ArmConstants.GRID_HIGH[0], ArmConstants.GRID_HIGH[1], 155),
                        new RunIntakeAction(0.5, -1),
                        new ArmHomeAction(),
                        new DrivePath(mOut),
                        //Parallel Out
                        new MoveThreePronged(
                                ArmConstants.INTER_FLOOR_INTAKE[0],
                                ArmConstants.INTER_FLOOR_INTAKE[1],
                                45,
                                ArmConstants.INTER2_FLOOR_INTAKE[0],
                                ArmConstants.INTER2_FLOOR_INTAKE[1],
                                90,
                                ArmConstants.FLOOR_INTAKE[0],
                                ArmConstants.FLOOR_INTAKE[1],
                                165),
                        new DrivePath(mReturn),
                        // Parallel In
                        new MoveTwoPronged(-ArmConstants.INTER_GRID_HIGH[0], ArmConstants.INTER_GRID_HIGH[1], 70,
                        -ArmConstants.GRID_HIGH[0], ArmConstants.GRID_HIGH[1], 155))));
    }

    @Override
    public void onEnded() {
    }

    @Override
    public String getName() {
        return "Cone Cube Sequence";
    }
}
