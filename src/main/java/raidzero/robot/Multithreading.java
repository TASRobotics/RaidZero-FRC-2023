package raidzero.robot;

import raidzero.robot.Constants.VisionConstants;
import raidzero.robot.submodules.Swerve;
import raidzero.robot.utils.MathTools;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Multithreading implements Runnable{

    private double timestamp;
    private NetworkTable cameraSubTable;
    private Pose2d newRobotPose;
    private Matrix<N3, N1> errors;
    // private Rotation2d cameraAngle;

    // private int[] aprilTagIDs;
    // private double[] xTranslationNT;
    // private double[] yTranslationNT;
    // private double[] zTranslationNT;
    // private double[] yawRotationNT;
    // private double[] confidenceNT;
    // private Pose2d[] aprilTagGlobalPoses;
    // private TimeInterpolatableBuffer<Rotation2d> angleInterpolate;

    private static final Swerve swerve = Swerve.getInstance();

    // public Multithreading(NetworkTable cameraSubTable, Pose2d cameraPose, Rotation2d cameraAngle, double timestamp){
    //     this.cameraSubTable = cameraSubTable;
    //     this.cameraPose = cameraPose;
    //     this.cameraAngle = cameraAngle;
    //     this.timestamp = timestamp;
    //     this.angleInterpolate = TimeInterpolatableBuffer.createBuffer(VisionConstants.ANGLEHISTSECS);
    // }

    public Multithreading(Pose2d newRobotPose, double timestamp, Matrix<N3, N1> errors) {
        this.newRobotPose = newRobotPose;
        this.timestamp = timestamp;
        this.errors = errors;
    }

    @Override
    public void run(){
        updatePose();
    }

    private void updatePose() {
        swerve.addVisionMeasurement(newRobotPose, timestamp,errors);
    }
}
