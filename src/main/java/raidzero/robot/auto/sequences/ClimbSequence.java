package raidzero.robot.auto.sequences;

import java.util.Arrays;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import raidzero.robot.Constants.SwerveConstants;
import raidzero.robot.submodules.Swerve;
import raidzero.robot.auto.actions.ArmHomeAction;
import raidzero.robot.auto.actions.DrivePath;
import raidzero.robot.auto.actions.SeriesAction;


public class ClimbSequence extends AutoSequence {
    private PathPlannerTrajectory mTrajectory = PathPlanner.loadPath("Balance", SwerveConstants.MAX_DRIVE_VEL_MPS * 0.75,
        SwerveConstants.MAX_DRIVE_ACCEL_MPSPS, false);
    
    @Override
    public void sequence() {
        addAction(
            new SeriesAction(Arrays.asList(
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
        return "Climb Sequence";
    }
}
