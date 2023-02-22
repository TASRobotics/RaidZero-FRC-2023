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
import raidzero.robot.auto.actions.SeriesAction;
import raidzero.robot.auto.actions.WaitAction;
import raidzero.robot.submodules.Intake;

public class SingleConeClimbSequence extends AutoSequence {
    private final static Intake mIntake = Intake.getInstance();
    
    private PathPlannerTrajectory mTrajectory = PathPlanner.loadPath("SCC Balance", SwerveConstants.MAX_DRIVE_VEL_MPS * 0.75,
        SwerveConstants.MAX_DRIVE_ACCEL_MPSPS, false);

    public SingleConeClimbSequence() {}

    @Override
    public void sequence() {
        addAction(
            new SeriesAction(Arrays.asList(
                new LambdaAction(() -> mIntake.setPercentSpeed(0.5)),
                new WaitAction(0.2), 
                new LambdaAction(() -> mIntake.setPercentSpeed(0)),
                new MoveTwoPronged(-.05, 1.5, 0, -ArmConstants.GRID_HIGH[0], ArmConstants.GRID_HIGH[1], 180),
                new LambdaAction(() -> mIntake.setPercentSpeed(-1)),
                new WaitAction(1), 
                new LambdaAction(() -> mIntake.setPercentSpeed(0)),
                new ArmHomeAction(), 
                new DrivePath(mTrajectory) 
            ))
        );
    }

    @Override
    public void onEnded() {

    }

    @Override
    public String getName() {
        return "Single Cone Climb Sequence";
    }
}
