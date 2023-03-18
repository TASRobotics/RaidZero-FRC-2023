package raidzero.robot.submodules;

import java.nio.file.Path;
import java.util.EnumSet;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BiFunction;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.ObjectMapper;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

// import com.fasterxml.jackson.annotation.JsonCreator;
// import com.fasterxml.jackson.annotation.JsonProperty;
// import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.estimator.AngleStatistics;
import edu.wpi.first.math.estimator.ExtendedKalmanFilter;
import edu.wpi.first.math.estimator.UnscentedKalmanFilter;
// import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer.InterpolateFunction;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
// import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.CircularBuffer;
import raidzero.robot.Constants;
import raidzero.robot.Constants.DriveConstants;
import raidzero.robot.Constants.VisionConstants;
import raidzero.robot.Constants.SwerveConstants;
import raidzero.robot.utils.MathTools;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision extends Submodule {

    private static final Swerve robotDrive = Swerve.getInstance();
    // private String tablename;
    private TimeInterpolatableBuffer<Rotation2d> angleInterpolate;
    // private UnscentedKalmanFilter<N2,N1,N1> aprilYawFilter;
    private double firsttimestamp;

    private Pose2d[] aprilTagGlobalPoses;
    private int[] aprilTagIDs;
    private double[] xTranslationNT;
    private double[] yTranslationNT;
    private double[] zTranslationNT;
    private double[] yawRotationNT;
    private double timestampNT;
    private String[] cameraSubTables;
    private double[] confidenceNT;

    private Pose2d robotPose;
    private Transform2d coneTransform;

    private NetworkTable table;
    // private final DoublePublisher timePublisher;
    // private final DoubleSubscriber timeSubscriber;



    private WPI_Pigeon2_Helper pigeon;

    private class WPI_Pigeon2_Helper extends WPI_Pigeon2 {
        public WPI_Pigeon2_Helper(int deviceNumber, String canbus) {
            super(deviceNumber, canbus);
        }

        public double getAngle() {
            return -super.getAngle();
        }
    }

    private static Vision instance;

    public static Vision getInstance() {
        if (instance == null) {
            instance = new Vision();
        }
        return instance;
    }

    private Vision() {
        robotPose = new Pose2d();
        coneTransform = new Transform2d();
        pigeon = new WPI_Pigeon2_Helper(SwerveConstants.IMU_ID, Constants.CANBUS_STRING);
        aprilTagGlobalPoses = GenerateAprilTagPoses(VisionConstants.APRILTAGPATH);
        int numAprilTags = aprilTagGlobalPoses.length;
        // angleInterpolate = TimeInterpolatableBuffer.createBuffer(VisionConstants.ANGLEHISTSECS);
        // angleHistory = new Rotation2d[VisionConstants.ANGLEHISTNUM];
        // timestampHistory = new double[VisionConstants.ANGLEHISTNUM];
        xTranslationNT = new double[numAprilTags];
        yTranslationNT = new double[numAprilTags];
        zTranslationNT = new double[numAprilTags];
        yawRotationNT = new double[numAprilTags];
        confidenceNT = new double[numAprilTags];
        
        // timePublisher = table.getDoubleTopic("RobotTime").publish();
        // timeSubscriber = table.getDoubleTopic("RobotTime").subscribe(0);
        // Nat<N2> states = Nat.N2();
        // ForwardState f = new ForwardState();
        // ForwardMeasure h = new ForwardMeasure();
        // Matrix<N2, N1> stateStdDevs = new MatBuilder<N2,N1>(Nat.N2(),
        // Nat.N1()).fill(0.05,0.005);
        // Matrix<N1, N1> measurementStdDevs = new MatBuilder<N1,N1>(Nat.N1(),
        // Nat.N1()).fill(0.1);
        // aprilYawFilter = new UnscentedKalmanFilter<N2,N1,N1>(Nat.N2(), Nat.N1(), f,
        // h, stateStdDevs, measurementStdDevs, Constants.TIMEOUT_S);
        // historyLoc = 1;
    }

    @Override
    public void onInit() {
        angleInterpolate = TimeInterpolatableBuffer.createBuffer(VisionConstants.ANGLEHISTSECS);
        
        // aprilYawFilter.reset();
        // for (String cameraSubTable : cameraSubTables) {
        //     table.getSubTable(cameraSubTable).addListener("Timestamp", EnumSet.of(NetworkTableEvent.Kind.kValueAll),
        //             (subTable, key, NetworkTableEvent) -> {
        //                 aprilDetect(subTable);
        //             });
        // }
    }

    @Override
    public void onStart(double timestamp) {
        System.out.println("Getting Cameras");
        cameraSubTables = getCameraNames();
        firsttimestamp = timestamp;
    }

    @Override
    public void update(double timestamp) {
        angleInterpolate.addSample(timestamp, robotDrive.getPose().getRotation());
        // updateRobotPose();
        // timePublisher.set(timestamp);
        SmartDashboard.putNumber("RobotTime", timestamp);
        
        for (String cameraSubTable : cameraSubTables) {
            aprilDetect(table.getSubTable(cameraSubTable));
        }
        if (robotPose != null) {
            SmartDashboard.putNumber("April Tag X Pose", robotPose.getX());
            SmartDashboard.putNumber("April Tag Y Pose", robotPose.getY());
            SmartDashboard.putNumber("Cone Translation", getConeTranslation());
        }

        // SmartDashboard.putBoolean("Apples?", !noApples());

        // table.putValue("April Tag X Pose", robotPose.getX());
        // table.putValue("April Tag X Pose", robotPose.getX());
        // aprilYawFilter.predict(new MatBuilder<N1,N1>(Nat.N1(),Nat.N1()).fill(0.0),
        // Constants.TIMEOUT_S);
        // angleHistory[historyLoc] = robotDrive.getPose().getRotation();
        // timestampHistory[historyLoc] = timestamp;
        // historyLoc = (historyLoc+1 == VisionConstants.ANGLEHISTNUM ? 0:
        // historyLoc+1);

    }

    private void aprilDetect(NetworkTable cameraSubTable) {
        // System.out.println("Detecting Apriltags");
        // System.out.println("Camera Subtable " +
        // cameraSubTable.getEntry("AprilTagIDs").getDoubleArray(new double[1])[0]);
        // cameraSubTable.getEntry("timestamp").clearFlags(EntryListenerFlags.kUpdate);
        aprilTagIDs = MathTools.doubleArraytoInt(cameraSubTable.getEntry("AprilTagIDs").getDoubleArray(new double[0]));
        xTranslationNT = cameraSubTable.getEntry("xTranslation").getDoubleArray(new double[aprilTagIDs.length]);
        yTranslationNT = cameraSubTable.getEntry("yTranslation").getDoubleArray(new double[aprilTagIDs.length]);
        zTranslationNT = cameraSubTable.getEntry("zTranslation").getDoubleArray(new double[aprilTagIDs.length]);
        yawRotationNT = cameraSubTable.getEntry("yawRotation").getDoubleArray(new double[aprilTagIDs.length]);
        confidenceNT = cameraSubTable.getEntry("Confidence").getDoubleArray(new double[aprilTagIDs.length]);
        int cameraNum = cameraSubTable.getPath().charAt(cameraSubTable.getPath().length() - 1) - '0';
        // System.out.println(cameraNum);
        // SmartDashboard.putNumber("Number of Seen Tags", aprilTagIDs.length);
        if (aprilTagIDs.length != 0)
            
            updatePose((new Pose2d()).plus(new Transform2d(VisionConstants.CAMERATRANSFORMS[cameraNum].getTranslation(), new Rotation2d())),
                VisionConstants.CAMERAANGLES[cameraNum],
                cameraSubTable.getEntry("Timestamp").getDouble(firsttimestamp));
    }

    // /**
    // * Helper method to get an entry from the Raspberry Pi NetworkTable.
    // *
    // * @param key Key for entry.
    // * @return NetworkTableEntry of given entry.
    // */
    private NetworkTableEntry getValue(String key) {
        if (table == null) {
            NetworkTableInstance.getDefault();
            table = NetworkTableInstance.getDefault().getTable(VisionConstants.NAME);

        }
        return table.getEntry(key);
    }



    private String[] getCameraNames() {
        return getValue("CameraNames").getStringArray(new String[] {"Camera 0", "Camera 1"});
    }

    // private void updatePose(double[] cameraTranslationZ, double[]
    // cameraTranslationX, double[] aTagRotation, int[] aTagIDs, double timestamp){
    // //Setup different poses of apriltags and robots relative to each other
    // Rotation2d aprilTagYaw;
    // // Pose2d aprilTagRelativePose;
    // // Rotation2d robotRelativeAngle;
    // Pose2d robotRelativePose;
    // Transform2d aprilTagRelativeTransformation;
    // Pose2d newRobotPose;
    // // int foundAngleLocation=-1;
    // // for(int i=historyLoc+timestampHistory.length-1;i>historyLoc;i--){
    // // if(timestamp>timestampHistory[i%timestampHistory.length])
    // foundAngleLocation = i%timestampHistory.length;
    // // }
    // // if (foundAngleLocation==-1) return;
    // //Interpolate between two rotation poses
    // // double interpolateTime =
    // (timestamp-timestampHistory[foundAngleLocation])/(timestampHistory[(foundAngleLocation+1)%timestampHistory.length]-timestampHistory[foundAngleLocation]);
    // for(int aTagID:aTagIDs){
    // // aTag2int = (int)aTagIDs[i];

    // //Create pose of robot with respect to the apriltag;
    // robotRelativePose = new Pose2d(-cameraTranslationZ[aTagID],
    // -cameraTranslationX[aTagID],new Rotation2d(aTagRotation[aTagID]));

    private void updatePose(Pose2d cameraPose, Rotation2d cameraAngle, double timestamp) {
        // Setup different poses of apriltags and robots relative to each other

        Pose2d newRobotPose;


        // int foundAngleLocation=-1;
        // for(int i=historyLoc+timestampHistory.length-1;i>historyLoc;i--){
        // if(timestamp>timestampHistory[i%timestampHistory.length]) foundAngleLocation
        // = i%timestampHistory.length;
        // }
        // if (foundAngleLocation==-1) return;
        // Interpolate between two rotation poses
        // double interpolateTime =
        // (timestamp-timestampHistory[foundAngleLocation])/(timestampHistory[(foundAngleLocation+1)%timestampHistory.length]-timestampHistory[foundAngleLocation]);
        for (int aTagID : aprilTagIDs) {
            // aTag2int = (int)aTagIDs[i];
            
            // Create pose of robot with respect to the apriltag;
            // robotRelativePose = new Pose2d(-cameraTranslationZ[aTagID], -cameraTranslationX[aTagID],
            //         new Rotation2d(aTagRotation[aTagID]));

            // Create rotation of apriltag with respect to the robot

            // Negative of Angle that robot must rotate to center the apriltag in vision
            // robotRelativeAngle = new Rotation2d(cameraTranslationZ[aTagID],
            // cameraTranslationX[aTagID]);

            // Create transformation of robot with respect to the apriltag, angle is the
            // corrected angle based on apriltag relative pose
            // SmartDashboard.putBoolean("Available Sample?", angleInterpolate.getSample(timestamp).isPresent());
            if (angleInterpolate.getSample(timestamp).isPresent() && confidenceNT[aTagID]>0) {
                // double pigeonAngle = pigeon.getAngle();
                // Rotation2d robotRotation = Rotation2d.fromDegrees(pigeonAngle);
    
                Pose2d aprilTagPose = aprilTagGlobalPoses[aTagID];
                Pose2d globalToAprilTag = new Pose2d(aprilTagPose.getTranslation(), angleInterpolate.getSample(timestamp).get().plus(cameraAngle));
    
                // Pose2d rotationPose = new Pose2d(0, 0, robotRotation);
                Transform2d aprilTagTransform = new Transform2d(
                        new Translation2d(-zTranslationNT[aTagID], xTranslationNT[aTagID]),
                        new Rotation2d());
                // SmartDashboard.putNumber("transform x", globalToAprilTag.getTranslation().getX());
                // SmartDashboard.putNumber("transform y", globalToAprilTag.getTranslation().getY());
                // SmartDashboard.putNumber("transform theta", globalToAprilTag.getRotation().getDegrees());
                // Pose2d cameraPose = rotationPose.plus(aprilTagTransform);
                // Transform2d aprilTagToCamera = new Transform2d(new Pose2d(), cameraPose);
                

            // Pose2d rotationPose = new Pose2d(0, 0, robotRotation);

                // SmartDashboard.putNumber("Transform Angle", globalToAprilTag.getRotation().getDegrees());
                Transform2d aprilToRobot = new Transform2d(new Pose2d(), cameraPose.plus(aprilTagTransform));
                // Transform2d aprilToRobot = new Transform2d(cameraPose.plus(aprilTagTransform).getTranslation(),cameraPose.plus(aprilTagTransform).getRotation());
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
            
                // System.out.println("Aligning with Apriltag " + aTagID);,
                if (newRobotPose.getTranslation().getDistance(new Translation2d()) >0 && newRobotPose.getTranslation().getDistance(new Translation2d())<30 ) {
                    // robotDrive.addVisionMeasurement(newRobotPose, timestamp,
                    //     new MatBuilder<N3, N1>(Nat.N3(), Nat.N1()).fill(positionError, positionError,
                    //             angleError));
                }
                
                
                SmartDashboard.putNumber("Camera Pose x", cameraPose.getTranslation().getX());
                SmartDashboard.putNumber("Camera Pose y", cameraPose.getTranslation().getY());
                SmartDashboard.putNumber("Camera Pose theta", cameraPose.getRotation().getDegrees());
                SmartDashboard.putNumber("Robot Relative Pose x", aprilToRobot.getTranslation().getX());
                SmartDashboard.putNumber("Robot Relative Pose y", aprilToRobot.getTranslation().getY());
                SmartDashboard.putNumber("Robot Relative Pose theta", aprilToRobot.getRotation().getDegrees());
                SmartDashboard.putNumber("Timestamp addition", timestamp);
                SmartDashboard.putNumber("Distance error", positionError);

    
                // aprilTagRelativeTransformation = new Transform2d(robotRelativePose.getTranslation(),
                //         aprilTagGlobalPoses[aTagID].getRotation().plus(aprilTagYaw)
                //                 .minus(angleInterpolate.getSample(timestamp).get()));

                // System.out.println("Calculating Pose: " +
                // angleInterpolate.getSample(timestamp));

                // newCameraPose = (new Pose2d(aprilTagGlobalPoses[aTagID].getTranslation(),
                //         angleInterpolate.getSample(timestamp).get())).plus(aprilTagRelativeTransformation);
                // newRobotPose = newCameraPose.plus(cameraPose);

            }


        }
    }

    // private Pose2d[] MakePose2dArray(double[] xcoords,double[] ycoords, double[]
    // angles){
    // Pose2d[] poses = new Pose2d[xcoords.length];
    // for (int i= 0;i<xcoords.length;i++){
    // poses[i] = new Pose2d(xcoords[i], ycoords[i], new Rotation2d(angles[i]));
    // }
    // return poses;
    // }

    @Override
    public void stop() {

    }

    // private static class ForwardState implements
    // BiFunction<Matrix<N2,N1>,Matrix<N1,N1>,Matrix<N2,N1>>{

    // private static Matrix<N2,N2> feedForward = new
    // MatBuilder<N2,N2>(Nat.N2(),Nat.N2()).fill(1.0,Constants.TIMEOUT_S,0.0,1.0);
    // @Override
    // public Matrix<N2,N1> apply(Matrix<N2,N1> x,Matrix<N1,N1> u){
    // return feedForward.times(x);
    // }
    // }
    // private static class ForwardMeasure implements
    // BiFunction<Matrix<N2,N1>,Matrix<N1,N1>,Matrix<N1,N1>>{

    // private static Matrix<N1,N2> feedForward = new
    // MatBuilder<N1,N2>(Nat.N1(),Nat.N2()).fill(1.0,0.0);
    // @Override
    // public Matrix<N1,N1> apply(Matrix<N2,N1> x,Matrix<N1,N1> u){
    // return feedForward.times(x);
    // }
    // }
    private static class AprilTagPoses extends Pose2d {
        private int ID;

        @JsonCreator
        public AprilTagPoses(@JsonProperty("x") double x, @JsonProperty("y") double y,
                @JsonProperty("angle") double angle, @JsonProperty("id") int id) {
            super(x, y, new Rotation2d(Math.toRadians(angle)));
            this.ID = id;
        }

        public int getID() {
            return ID;
        }

    }

    public Pose2d[] GenerateAprilTagPoses(Path aprilTagPath) {
        AprilTagPoses[] aPoses = null;
        try {
            ObjectMapper mapper = new ObjectMapper();
            aPoses = mapper.readValue(aprilTagPath.toFile(), AprilTagPoses[].class);
        } catch (Exception e1) {
            System.out.println("Cannot parse file");
            e1.printStackTrace();
        }
        // int maxTags = 0;
        // for (AprilTagPoses element : aPoses) {
        // maxTags = element.getID()>maxTags ? element.getID() : maxTags;
        // }
        // List<AprilTagPoses> outPoses = new ArrayList<AprilTagPoses>(maxTags);
        // AprilTagPoses[] finalPoses = new AprilTagPoses[aPoses.length];
        // for (int i = 0;i<aPoses.length;i++){
        // finalPoses[aPoses[i].getID()] = aPoses[i];
        // }
        for (int i = 0; i < aPoses.length; i++) {
            System.out.println("Tag number " + i + ": " + aPoses[i].getX() + " " + aPoses[i].getY() + " "
                    + aPoses[i].getRotation().getDegrees());
        }

        return aPoses;
    }

    public void updateRobotPose() {
        NetworkTable subTable = table.getSubTable("Camera 1");
        aprilDetect(subTable);
        if (aprilTagIDs.length > 0) {
            double pigeonAngle = pigeon.getAngle();
            Rotation2d robotRotation = Rotation2d.fromDegrees(pigeonAngle);

            Pose2d aprilTagPose = aprilTagGlobalPoses[aprilTagIDs[0]];
            Pose2d globalToAprilTag = new Pose2d(aprilTagPose.getTranslation(), robotRotation);

            // Pose2d rotationPose = new Pose2d(0, 0, robotRotation);
            Transform2d aprilTagTransform = new Transform2d(
                    new Translation2d(-zTranslationNT[aprilTagIDs[0]], xTranslationNT[aprilTagIDs[0]]),
                    new Rotation2d());
            // SmartDashboard.putNumber("transform x", globalToAprilTag.getTranslation().getX());
            // SmartDashboard.putNumber("transform y", globalToAprilTag.getTranslation().getY());
            // SmartDashboard.putNumber("transform theta", globalToAprilTag.getRotation().getDegrees());
            // Pose2d cameraPose = rotationPose.plus(aprilTagTransform);
            // Transform2d aprilTagToCamera = new Transform2d(new Pose2d(), cameraPose);

            Transform2d cameraToRobot = VisionConstants.CAMERATRANSFORMS[0];

            Transform2d aprilToRobot = aprilTagTransform.plus(cameraToRobot);
            robotPose = globalToAprilTag.plus(aprilToRobot);

            // Pose2d finalPose = new
            // Pose2d().transformBy(globalToAprilTag).transformBy(aprilTagToCamera).transformBy(cameraToRobot);
            // robotPose = new Pose2d(finalPose.getX(), finalPose.getY(),
            // robotDrive.getPose().getRotation());
        } else {
            robotPose = null;
        }
    }

    public Pose2d getRobotPose() {
        return robotPose;
    }

    /**
     * Checks whether the camera is currently seeing an April Tag
     * 
     * @return whether aprilTagIds = 0
     */
    public boolean noApples() {
        // return aprilTagIDs.length == 0;
        return false;
    }

    public double getConeTranslation() {
        return table.getSubTable("Camera 0").getEntry("X Translation").getDouble(0)
                * VisionConstants.CONE_PIXELS_TO_METERS;
    }

    public Transform2d getConeTransform(){
        double pigeonAngle = pigeon.getAngle();
        Rotation2d robotRotation = Rotation2d.fromDegrees(pigeonAngle);

        if (robotRotation.getDegrees() < 90 && robotRotation.getDegrees() > -90){
            return new Transform2d(new Translation2d(0, getConeTranslation()), new Rotation2d());
        }
        else{
            return new Transform2d(new Translation2d(0, -getConeTranslation()), new Rotation2d());
        }
    }
}
