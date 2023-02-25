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
import raidzero.robot.auto.actions.MoveTwoPronged;
import raidzero.robot.auto.actions.RunIntakeAction;
import raidzero.robot.auto.actions.SeriesAction;
import raidzero.robot.submodules.Swerve;

public class SingleConeClimbSequence extends AutoSequence {
    private static final Swerve mSwerve = Swerve.getInstance();

    private PathPlannerTrajectory mOverRamp = PathPlanner.loadPath("SCC Over", SwerveConstants.MAX_DRIVE_VEL_MPS * 0.5,
            SwerveConstants.MAX_DRIVE_ACCEL_MPSPS * 0.5);
    private PathPlannerTrajectory mBalance = PathPlanner.loadPath("SCC Balance", SwerveConstants.MAX_DRIVE_VEL_MPS * 0.5,
            SwerveConstants.MAX_DRIVE_ACCEL_MPSPS * 0.5);

    public SingleConeClimbSequence() {
        PathPlannerTrajectory.transformTrajectoryForAlliance(mOverRamp, DriverStation.getAlliance());
        PathPlannerTrajectory.transformTrajectoryForAlliance(mBalance, DriverStation.getAlliance());
    }

    @Override
    public void sequence() {
        addAction(
            new SeriesAction(Arrays.asList(
                new RunIntakeAction(0.2, 0.5),
                new MoveTwoPronged(-ArmConstants.INTER_GRID_HIGH[0], ArmConstants.INTER_GRID_HIGH[1], 70, -ArmConstants.GRID_HIGH[0], ArmConstants.GRID_HIGH[1], 155),
                new RunIntakeAction(1, -1),
                new ArmHomeAction(),
                new DrivePath(mOverRamp),
                new DrivePath(mBalance), 
                new LambdaAction(() -> mSwerve.rotorBrake(true)))));
    }

    @Override
    public void onEnded() {}

    @Override
    public String getName() {
        return "Single Cone Climb Sequence";
    }
}
