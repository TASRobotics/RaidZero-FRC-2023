package raidzero.robot.auto.sequences;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import raidzero.robot.Constants.SwerveConstants;
import raidzero.robot.auto.actions.DrivePath;

public class TestSequence extends AutoSequence {
    private PathPlannerTrajectory mTrajectory = PathPlanner.loadPath("TestPath", SwerveConstants.MAX_DRIVE_VEL, SwerveConstants.MAX_DRIVE_ACCEL);

    public TestSequence() {}

    @Override
    public void sequence() {
        addAction(
            new DrivePath(mTrajectory)
        );
    }

    @Override
    public void onEnded() {
        System.out.println("TestSequence ended!");
    }

    @Override
    public String getName() {
        return "Test Sequence";
    }
}
