package raidzero.robot.utils;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import raidzero.robot.submodules.Swerve;

public class AutoAimController {
    private PIDController mXController, mYController;
    private ProfiledPIDController mThetaController;
    private TrajectoryConfig mTrajectoryConfig;
    private Pose2d mPoseError = new Pose2d();
    private Rotation2d mRotationError = new Rotation2d();
    private Timer mTimer = new Timer();

    private Trajectory mTrajectory;
    private Rotation2d mEndHeading;
    private static final Swerve mSwerve = Swerve.getInstance();

    /**
     * Create new Auto Aim Controller
     * 
     * @param xController x PID controller (meters)
     * @param yController y PID controller (meters)
     * @param thetaController theta PID controller (radians)
     */
    public AutoAimController(
        PIDController xController, 
        PIDController yController, 
        ProfiledPIDController thetaController, 
        TrajectoryConfig trajectoryConfig
    ) {
        mXController = xController;
        mYController = yController;
        mThetaController = thetaController;
        mTrajectoryConfig = trajectoryConfig;
    }

    public void setTarget(Pose2d currPose, List<Translation2d> interPoints, Pose2d finalPose, Rotation2d endHeading) {
        Trajectory traj = TrajectoryGenerator.generateTrajectory(
            currPose, 
            interPoints, 
            finalPose, 
            mTrajectoryConfig
        );
        followPath(traj, endHeading);
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
        ChassisSpeeds speeds = calculate(mSwerve.getPose(), currState, mEndHeading);
        mSwerve.setOpenLoopSpeeds(speeds);
    }

    /**
     * Calculate output speeds
     * 
     * @param currPose current robot pose (meters)
     * @param trajState desired trajectory state (meters)
     * @param desiredHeading desired final heading
     * @return output chassis speeds
     */
    private ChassisSpeeds calculate(Pose2d currPose, Trajectory.State trajState, Rotation2d desiredHeading) {
        double xFF = trajState.velocityMetersPerSecond * trajState.poseMeters.getRotation().getCos();
        double yFF = trajState.velocityMetersPerSecond * trajState.poseMeters.getRotation().getSin();
        double thetaFF =
            mThetaController.calculate(
                currPose.getRotation().getRadians(), desiredHeading.getRadians());
    
        mPoseError = trajState.poseMeters.relativeTo(currPose);
        mRotationError = desiredHeading.minus(currPose.getRotation());

        double xFeedback = mXController.calculate(currPose.getX(), trajState.poseMeters.getX());
        double yFeedback = mYController.calculate(currPose.getY(), trajState.poseMeters.getY());
    
        // Return next output.
        return ChassisSpeeds.fromFieldRelativeSpeeds(
            xFF + xFeedback, 
            yFF + yFeedback, 
            thetaFF, 
            currPose.getRotation()
        );
    }

    /**
     * Get positional error
     * 
     * @return positional error (meters)
     */
    public Pose2d getError() {
        return new Pose2d(mPoseError.getX(), mPoseError.getY(), mRotationError);
    }

    /**
     * Set tolerance
     * 
     * @param tolerance tolerance (meters & radians)
     */
    public void setTolerance(Pose2d tolerance) {
        mXController.setTolerance(tolerance.getX());
        mYController.setTolerance(tolerance.getY());
        mThetaController.setTolerance(tolerance.getRotation().getRadians());
    }

    /**
     * Check if robot is at target
     * 
     * @return robot at target
     */
    public boolean atTarget() {
        return mXController.atSetpoint() && mYController.atSetpoint() && mThetaController.atSetpoint();
    }
}
