package raidzero.robot.auto.sequences;

import java.util.Arrays;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import raidzero.robot.Constants.SwerveConstants;
import raidzero.robot.auto.actions.DrivePath;
import raidzero.robot.auto.actions.LambdaAction;
import raidzero.robot.auto.actions.ParallelAction;
import raidzero.robot.auto.actions.SeriesAction;
import raidzero.robot.auto.actions.WaitForEventMarkerAction;
import raidzero.robot.submodules.Intake;
import raidzero.robot.submodules.Swerve;

public class TestSequence extends AutoSequence {
    private PathPlannerTrajectory mTrajectory = PathPlanner.loadPath("Straight Path", SwerveConstants.MAX_DRIVE_VEL_MPS,
            SwerveConstants.MAX_DRIVE_ACCEL_MPSPS);
    private Swerve mSwerve = Swerve.getInstance();
    private Intake mIntake = Intake.getInstance();


    @Override
    public void sequence() {
        // addAction(
        //     new ParallelAction(Arrays.asList(
        //         new DrivePath(mTrajectory), 
        //         new SeriesAction(Arrays.asList(
        //             new WaitForEventMarkerAction(mTrajectory, "deez nuts", mSwerve.getPathingTime()), 
        //             new LambdaAction(() -> mIntake.set(0.5)))
        //         ))
        //     )
        // );
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
