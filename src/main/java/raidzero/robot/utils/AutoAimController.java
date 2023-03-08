package raidzero.robot.utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;

public class AutoAimController {
    private PIDController mXController, mYController;
    private ProfiledPIDController mThetaController;
    private Pose2d mPoseError = new Pose2d();
    private Rotation2d mRotationError = new Rotation2d();

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
        ProfiledPIDController thetaController
    ) {
        mXController = xController;
        mYController = yController;
        mThetaController = thetaController;
    }

    /**
     * Calculate output speeds
     * 
     * @param currPose current robot pose (meters)
     * @param trajState desired trajectory state (meters)
     * @param desiredHeading desired final heading
     * @return output chassis speeds
     */
    public ChassisSpeeds calculate(Pose2d currPose, Trajectory.State trajState, Rotation2d desiredHeading) {
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
