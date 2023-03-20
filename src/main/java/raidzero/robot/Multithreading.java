package raidzero.robot;

import raidzero.robot.Constants.VisionConstants;
import raidzero.robot.submodules.Swerve;
import raidzero.robot.utils.MathTools;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.networktables.NetworkTable;

public class Multithreading implements Runnable{

    private double timestamp;
    private NetworkTable cameraSubTable;
    private Pose2d cameraPose;
    private Rotation2d cameraAngle;

    private int[] aprilTagIDs;
    private double[] xTranslationNT;
    private double[] yTranslationNT;
    private double[] zTranslationNT;
    private double[] yawRotationNT;
    private double[] confidenceNT;
    private Pose2d[] aprilTagGlobalPoses;
    private TimeInterpolatableBuffer<Rotation2d> angleInterpolate;

    private static final Swerve swerve = Swerve.getInstance();

    public Multithreading(NetworkTable cameraSubTable, Pose2d cameraPose, Rotation2d cameraAngle, double timestamp){
        this.cameraSubTable = cameraSubTable;
        this.cameraPose = cameraPose;
        this.cameraAngle = cameraAngle;
        this.timestamp = timestamp;
        this.angleInterpolate = TimeInterpolatableBuffer.createBuffer(VisionConstants.ANGLEHISTSECS);
    }

    @Override
    public void run(){
        updatePose();
    }

    private void updatePose() {
        aprilTagIDs = MathTools.doubleArraytoInt(cameraSubTable.getEntry("AprilTagIDs").getDoubleArray(new double[0]));
        xTranslationNT = cameraSubTable.getEntry("xTranslation").getDoubleArray(new double[aprilTagIDs.length]);
        yTranslationNT = cameraSubTable.getEntry("yTranslation").getDoubleArray(new double[aprilTagIDs.length]);
        zTranslationNT = cameraSubTable.getEntry("zTranslation").getDoubleArray(new double[aprilTagIDs.length]);
        yawRotationNT = cameraSubTable.getEntry("yawRotation").getDoubleArray(new double[aprilTagIDs.length]);
        confidenceNT = cameraSubTable.getEntry("Confidence").getDoubleArray(new double[aprilTagIDs.length]);

        Pose2d newRobotPose;

        for (int aTagID : aprilTagIDs) {
            if (angleInterpolate.getSample(timestamp).isPresent() && confidenceNT[aTagID]>0) {

                Pose2d aprilTagPose = aprilTagGlobalPoses[aTagID];
                Pose2d globalToAprilTag = new Pose2d(aprilTagPose.getTranslation(), angleInterpolate.getSample(timestamp).get().plus(cameraAngle));
    
                Transform2d aprilTagTransform = new Transform2d(
                        new Translation2d(-zTranslationNT[aTagID], xTranslationNT[aTagID]),
                        new Rotation2d());

                Transform2d aprilToRobot = new Transform2d(new Pose2d(), cameraPose.plus(aprilTagTransform));
                newRobotPose = globalToAprilTag.plus(aprilToRobot);
                Rotation2d measuredYaw = aprilTagGlobalPoses[aTagID].getRotation().plus(new Rotation2d(Math.toRadians(yawRotationNT[aTagID])).minus(cameraAngle));

                newRobotPose = new Pose2d(newRobotPose.getTranslation(), measuredYaw);
                double positionError;
                double angleError;
                try{
                    positionError = VisionConstants.DISTANCEERRORFACTOR*Math.abs(Math.pow(zTranslationNT[aTagID],2))/confidenceNT[aTagID];
                    angleError = VisionConstants.ANGLEERRORFACTOR*Math.abs(zTranslationNT[aTagID])/confidenceNT[aTagID];
                } catch (Exception e){
                    positionError = 1;
                    angleError = 10;
                }
            
                if (newRobotPose.getTranslation().getDistance(new Translation2d()) >0 && newRobotPose.getTranslation().getDistance(new Translation2d())<30 ) {
                    swerve.addVisionMeasurement(newRobotPose, timestamp,
                        new MatBuilder<N3, N1>(Nat.N3(), Nat.N1()).fill(positionError, positionError,
                                angleError));
                }
            }
        }
    }
}
