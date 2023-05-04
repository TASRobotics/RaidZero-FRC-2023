package raidzero.robot.auto.actions;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import raidzero.robot.submodules.Swerve;
import raidzero.robot.submodules.Vision;
import raidzero.robot.utils.VisionPose2d;

/**
 * Action for following a path.
 */
public class CubeAlighmentAction implements Action {

    private static final Swerve swerve = Swerve.getInstance();
    private Transform2d transform;
    private Pose2d updatePose;
    private Pose2d endPose;
    private Rotation2d cubeAngle;
    private Timer timer = new Timer();
    private Vision vision = Vision.getInstance();

    public CubeAlighmentAction(Pose2d endPose) {
        this.endPose = endPose;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void start() {
        timer.reset();
        timer.start();
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' started!");
    }

    @Override
    public void update() {
        if (vision.isCube() && vision.getCubePercentArea() < 0.5){
            transform = swerve.getPose().minus(endPose);
            // Translation2d relativeTranslation = transform.getTranslation();

            // Rotation2d relativeAngle = transform.getRotation();
            // Rotation2d angleDiff = cubeAngle.minus(relativeAngle);
            
            cubeAngle = new Rotation2d(Math.toRadians(vision.getCubeAngle()));
            Rotation2d angleDiff = new Rotation2d(Math.PI).minus(transform.getTranslation().getAngle()).plus(transform.getRotation()).minus(cubeAngle);
            
            Transform2d angleTransform = new Transform2d(new Translation2d(), angleDiff);

            // transform = transform.plus(new Transform2d(new Translation2d(), transform.getRotation().minus(cubeAngle)));
            // updatePose = swerve.getPose().plus(transform);
            // Pose2d relativeRobotPose = new Pose2d(relativeTranslation, cubeAngle);
            
            // Pose2d newCubePose = new Pose2d(endPose.getTranslation(), endPose.getRotation().plus(angleDiff));
            // Transform2d newRobotTransform = new Transform2d(transform.getTranslation(), transform.getRotation().minus(angleDiff));
            
            Pose2d newRobotPose = endPose.plus(angleTransform).plus(transform).plus(angleTransform.inverse());  //newCubePose.plus(newRobotTransform);
            VisionPose2d addingPose = new VisionPose2d(newRobotPose, Timer.getFPGATimestamp(), 0.01  / (Math.abs(vision.getCubePercentArea()) + 1));
            swerve.addVisionMeasurement(addingPose);
        }
    }

    @Override
    public void done() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' finished!");
        timer.stop();
    }
}
