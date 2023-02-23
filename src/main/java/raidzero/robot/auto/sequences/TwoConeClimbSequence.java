package raidzero.robot.auto.sequences;

import java.util.Arrays;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import raidzero.robot.Constants.SwerveConstants;
import raidzero.robot.auto.actions.DrivePath;
import raidzero.robot.auto.actions.LambdaAction;
import raidzero.robot.auto.actions.MoveTwoPronged;
import raidzero.robot.auto.actions.ParallelAction;
import raidzero.robot.auto.actions.SeriesAction;
import raidzero.robot.auto.actions.WaitAction;
import raidzero.robot.auto.actions.MoveTwoPronged;
import raidzero.robot.auto.actions.WaitForEventMarkerAction;
import raidzero.robot.auto.actions.ArmHomeAction;
import edu.wpi.first.wpilibj.Timer;
import raidzero.robot.submodules.Intake;
import raidzero.robot.submodules.Swerve;
import raidzero.robot.submodules.Arm;
import raidzero.robot.Constants.ArmConstants;;

public class TwoConeClimbSequence extends AutoSequence {
    private PathPlannerTrajectory mOverRamp = PathPlanner.loadPath("TCC Over", SwerveConstants.MAX_DRIVE_VEL_MPS,
            SwerveConstants.MAX_DRIVE_ACCEL_MPSPS);
    private PathPlannerTrajectory mBalance = PathPlanner.loadPath("TCC Balance", SwerveConstants.MAX_DRIVE_VEL_MPS,
            SwerveConstants.MAX_DRIVE_ACCEL_MPSPS);

    private static final Swerve mSwerve = Swerve.getInstance();
    private static final Arm mArm = Arm.getInstance();
    private static final Intake mIntake = Intake.getInstance();

    @Override
    public void sequence() {
        addAction(
                new SeriesAction(Arrays.asList(
                        // Score preload in mid rung
                        // new MoveTwoPronged(-.05, 1.5, 0, -ArmConstants.GRID_HIGH[0],
                        // ArmConstants.GRID_HIGH[1], 180),
                        // new LambdaAction(() -> mIntake.setPercentSpeed(-1)),
                        // new WaitAction(0.5),
                        // new LambdaAction(() -> mIntake.setPercentSpeed(0)),
                        // new ArmHomeAction()

                        // Climb over charge station & get cone
                        new DrivePath(mOverRamp),
                        new WaitAction(1),
                        new DrivePath(mBalance)

                // // Climb Ramp
                // new ParallelAction(Arrays.asList(
                // new DrivePath(mRClimbRamp),
                // new LambdaAction(() -> {
                // mIntake.setPercentSpeed(0);
                // }),
                // new SeriesAction(Arrays.asList(
                // new WaitForEventMarkerAction(mRClimbRamp, "",
                // mSwerve.getPathingTime()),
                // new LambdaAction(() -> mIntake.setPercentSpeed(0.5)))))),
                // // Floor Pickup
                // new LambdaAction(() -> {
                // mIntake.setPercentSpeed(0.3);
                // mArm.moveToPoint(ArmConstants.FLOOR_INTAKE[0], ArmConstants.FLOOR_INTAKE[1],
                // 180);
                // Timer.delay(4);
                // mIntake.holdPosition();
                // Timer.delay(0.5);
                // mArm.goHome();
                // }),
                // // Reverse Climb Ramp
                // new ParallelAction(Arrays.asList(
                // new DrivePath(mSClimbRamp),
                // new LambdaAction(() -> {
                // // Timer.delay(1);
                // }))),
                // // Score High
                // new LambdaAction(() -> {
                // mArm.moveTwoPronged(-0.05, 0.8, 0, -ArmConstants.GRID_HIGH[0],
                // ArmConstants.GRID_HIGH[1], 180);
                // Timer.delay(5);
                // mIntake.setPercentSpeed(-0.7);
                // Timer.delay(0.5);
                // mArm.goHome();
                // }),
                // // Balance
                // new ParallelAction(Arrays.asList(
                // new DrivePath(mBalance),
                // new LambdaAction(() -> {
                // // Timer.delay(1);
                // })))

                )));
        System.out.println("Added actions.");
    }

    @Override
    public void onEnded() {
        System.out.println("Two Cone Climb ended!");
    }

    @Override
    public String getName() {
        return "Two Cone Climb Sequence";
    }
}
