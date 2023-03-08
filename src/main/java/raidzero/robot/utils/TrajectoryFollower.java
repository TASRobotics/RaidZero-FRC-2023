package raidzero.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import raidzero.robot.submodules.Swerve;

public class TrajectoryFollower {
    private AutoAimController mController;
    private Trajectory mTrajectory;
    private Rotation2d mEndHeading;
    private Timer mTimer = new Timer();
    private static final Swerve mSwerve = Swerve.getInstance();

    /**
     * Create Trajectory Follower (made for auto aim)
     * 
     * @param controller {@link} AutoAimController
     */
    public TrajectoryFollower(AutoAimController controller) {
        mController = controller;
    }

    /**
     * Follow Trajectory
     * 
     * @param trajectory desired trajectory
     * @param endHeading desired end heading
     */
    public void followPath(Trajectory trajectory, Rotation2d endHeading) {
        mTimer.reset();
        mTimer.start();
        mTrajectory = trajectory;
        mEndHeading = endHeading;
    }

    /** Update Controller */
    public void update() {
        Trajectory.State currState = mTrajectory.sample(mTimer.get());
        ChassisSpeeds speeds = mController.calculate(mSwerve.getPose(), currState, mEndHeading);
        mSwerve.setOpenLoopSpeeds(speeds);
    }

    /**
     * Check if finished
     * 
     * @return check finished 
     */
    public boolean isFinished() {
        return mController.atTarget();
    }
}
