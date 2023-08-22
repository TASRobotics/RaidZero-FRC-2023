package raidzero.robot.submodules;

import java.nio.file.Path;
// import java.util.Arrays;
// import java.util.EnumSet;
import java.util.LinkedList;
// import java.util.Objects;
// import java.util.concurrent.BlockingDeque;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
// import java.util.concurrent.ThreadPoolExecutor;
// import java.util.concurrent.atomic.AtomicReference;
// import java.util.function.BiFunction;
// import java.util.function.DoubleToIntFunction;
// import java.util.stream.DoubleStream;
// import java.util.stream.IntStream;

// import javax.annotation.processing.Generated;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.databind.ObjectMapper;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

// import com.fasterxml.jackson.annotation.JsonCreator;
// import com.fasterxml.jackson.annotation.JsonProperty;
// import com.fasterxml.jackson.databind.ObjectMapper;

// import edu.wpi.first.math.MatBuilder;
// import edu.wpi.first.math.Matrix;
// import edu.wpi.first.math.Nat;
// import edu.wpi.first.math.Num;
// import edu.wpi.first.math.estimator.AngleStatistics;
// import edu.wpi.first.math.estimator.ExtendedKalmanFilter;
// import edu.wpi.first.math.estimator.UnscentedKalmanFilter;
// import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer.InterpolateFunction;
// import edu.wpi.first.math.numbers.N1;
// import edu.wpi.first.math.numbers.N2;
// import edu.wpi.first.math.numbers.N3;
// import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.networktables.DoubleArraySubscriber;
// import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.IntegerArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.CircularBuffer;
import raidzero.robot.Constants;
import raidzero.robot.Constants.DriveConstants;
import raidzero.robot.Constants.PoseConstants;
import raidzero.robot.Constants.VisionConstants;
import raidzero.robot.Constants.SwerveConstants;
import raidzero.robot.utils.MathTools;
import raidzero.robot.utils.ParallelPoseAdding;
import raidzero.robot.utils.VisionPose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision extends Submodule {

    private static final Swerve robotDrive = Swerve.getInstance();
    private static final Lights lights = Lights.getInstance();
    // private String tablename;
    private TimeInterpolatableBuffer<Rotation2d> angleInterpolate;
    // private UnscentedKalmanFilter<N2,N1,N1> aprilYawFilter;
    private double firsttimestamp;

    private double poseCounter;

    private Pose2d[] aprilTagGlobalPoses;

    private DoubleArraySubscriber[] xApril;
    private DoubleArraySubscriber[] zApril;
    private DoubleArraySubscriber[] yawApril;
    private DoubleArraySubscriber[] confidenceApril;
    private DoubleSubscriber[] timeApril;
    private DoubleArraySubscriber[] tagsApril;

    private int[][] aprilTagIDs;
    private double[][] xTranslationNT;
    // private double[] yTranslationNT;
    private double[][] zTranslationNT;
    private double[][] yawRotationNT;
    private double[] timestampNT;
    private String[] cameraSubTables;
    private double[][] confidenceNT;

    private Pose2d robotPose;
    private Transform2d coneTransform;

    private double cubeX;
    private double cubeY;
    private double cubeAngle;
    private double cubePercentArea;

    private NetworkTable table;
    // private final DoublePublisher timePublisher;
    // private final DoubleSubscriber timeSubscriber;

    private BlockingQueue<Thread> blockingQueue = new LinkedBlockingQueue<>(VisionConstants.NUM_THREADS);

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
        // angleInterpolate =
        // TimeInterpolatableBuffer.createBuffer(VisionConstants.ANGLEHISTSECS);
        // angleHistory = new Rotation2d[VisionConstants.ANGLEHISTNUM];
        // timestampHistory = new double[VisionConstants.ANGLEHISTNUM];
        // xTranslationNT = new double[numAprilTags];
        // yTranslationNT = new double[numAprilTags];
        // zTranslationNT = new double[numAprilTags];
        // yawRotationNT = new double[numAprilTags];
        // confidenceNT = new double[numAprilTags];

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
        // table.getSubTable(cameraSubTable).addListener("Timestamp",
        // EnumSet.of(NetworkTableEvent.Kind.kValueAll),
        // (subTable, key, NetworkTableEvent) -> {
        // aprilDetect(subTable);
        // });
        // }
    }

    @Override
    public void onStart(double timestamp) {
        System.out.println("Getting Cameras");
        cameraSubTables = getCameraNames();
        xApril = new DoubleArraySubscriber[cameraSubTables.length];
        zApril = new DoubleArraySubscriber[cameraSubTables.length];
        yawApril = new DoubleArraySubscriber[cameraSubTables.length];
        confidenceApril = new DoubleArraySubscriber[cameraSubTables.length];
        timeApril = new DoubleSubscriber[cameraSubTables.length];

        for(int cameraNum = 0; cameraNum<cameraSubTables.length; cameraNum++){
            timeApril[cameraNum] = table.getSubTable(cameraSubTables[cameraNum]).getDoubleTopic("Timestamp").subscribe(0);
            xApril[cameraNum] = table.getSubTable(cameraSubTables[cameraNum]).getDoubleArrayTopic("xTranslation").subscribe(new double[aprilTagGlobalPoses.length]);
            zApril[cameraNum] = table.getSubTable(cameraSubTables[cameraNum]).getDoubleArrayTopic("zTranslation").subscribe(new double[aprilTagGlobalPoses.length]);
            yawApril[cameraNum] = table.getSubTable(cameraSubTables[cameraNum]).getDoubleArrayTopic("yawRotation").subscribe(new double[aprilTagGlobalPoses.length]);
            confidenceApril[cameraNum] = table.getSubTable(cameraSubTables[cameraNum]).getDoubleArrayTopic("yawRotation").subscribe(new double[aprilTagGlobalPoses.length]);            
        }
        firsttimestamp = timestamp;
    }

    @Override
    public void update(double timestamp) {
        angleInterpolate.addSample(timestamp, robotDrive.getPose().getRotation());
        // updateRobotPose();
        // timePublisher.set(timestamp);
        SmartDashboard.putNumber("RobotTime", timestamp);

        updateCubeValues();

        // for (int cameraNum = 0; cameraNum<cameraSubTables.length;cameraNum++) {
        // aprilDetect();
        // }
        if (robotPose != null) {
            SmartDashboard.putNumber("April Tag X Pose", robotPose.getX());
            SmartDashboard.putNumber("April Tag Y Pose", robotPose.getY());
            SmartDashboard.putNumber("Cube X", getCubeX());
            SmartDashboard.putNumber("Cube Y", getCubeY());
            SmartDashboard.putNumber("Cube Angle", getCubeAngle());
        }

        //SmartDashboard.putBoolean("Apples?", !noApples());

        // table.putValue("April Tag X Pose", robotPose.getX());
        // table.putValue("April Tag X Pose", robotPose.getX());
        // aprilYawFilter.predict(new MatBuilder<N1,N1>(Nat.N1(),Nat.N1()).fill(0.0),
        // Constants.TIMEOUT_S);
        // angleHistory[historyLoc] = robotDrive.getPose().getRotation();
        // timestampHistory[historyLoc] = timestamp;
        // historyLoc = (historyLoc+1 == VisionConstants.ANGLEHISTNUM ? 0:
        // historyLoc+1);

    }

    private int nonZeroDouble(double test){
        return Math.abs(test)<0.01? 0 : 1;
    }

    private void aprilDetect() {
        // System.out.println("Detecting Apriltags");
        // System.out.println("Camera Subtable " +
        // cameraSubTable.getEntry("AprilTagIDs").getDoubleArray(new double[1])[0]);
        // cameraSubTable.getEntry("timestamp").clearFlags(EntryListenerFlags.kUpdate);
        LinkedList<VisionPose2d> posesToAdd = new LinkedList<VisionPose2d>();

        for (int cameraNum = 0; cameraNum<cameraSubTables.length;cameraNum++) {
            aprilTagIDs = MathTools.doubleArraytoInt(tagsApril[cameraNum].readQueueValues());
            xTranslationNT = xApril[cameraNum].readQueueValues();
            zTranslationNT = zApril[cameraNum].readQueueValues();
            yawRotationNT = yawApril[cameraNum].readQueueValues();
            confidenceNT = confidenceApril[cameraNum].readQueueValues();
            timestampNT = timeApril[cameraNum].readQueueValues();
        

        // System.out.println(cameraNum);
        // SmartDashboard.putNumber("Number of Seen Tags", aprilTagIDs.length);
        // if(confidenceNT.length ==0) return null;
            // int totalPoses = 0;
            // for(int[] aprilTagIDhist:aprilTagIDs){
            //     totalPoses+=aprilTagIDhist.length;
            // }
            

            for(int queuePos = 0; queuePos<confidenceNT.length;queuePos++){
                
                for(int aprilID : aprilTagIDs[queuePos]){
                    
                    Pose2d toAdd = constructPose(xTranslationNT[queuePos][aprilID],
                    zTranslationNT[queuePos][aprilID],yawRotationNT[queuePos][aprilID],
                    aprilID, timestampNT[queuePos], cameraNum);

                    if(toAdd != null) posesToAdd.add(new VisionPose2d(
                        toAdd, timestampNT[queuePos],
                        zTranslationNT[queuePos][aprilID]/confidenceNT[queuePos][aprilID]));
                }
                
            }
        }
        // Need to implement this with errors and timestamps effectively
        

        ParallelPoseAdding addPoses = new ParallelPoseAdding(posesToAdd);
        addPoses.run();
        // updatePose(
        //         (new Pose2d()).plus(new Transform2d(
        //                 VisionConstants.CAMERATRANSFORMS[cameraNum].getTranslation(), new Rotation2d())),
        //         VisionConstants.CAMERAANGLES[cameraNum],
        //         cameraSubTable.getEntry("Timestamp").getDouble(firsttimestamp));
        
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

    private NetworkTableEntry getSubValue(String key, String subtable) {
        if (table == null) {
            NetworkTableInstance.getDefault();
            table = NetworkTableInstance.getDefault().getTable(VisionConstants.NAME);

        }
        return table.getSubTable(subtable).getEntry(key);
    }

    private String[] getCameraNames() {
        return getValue("CameraNames").getStringArray(new String[] { "Camera 0", "Camera 1" });
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


    /**
     * Constructs the pose of the robot given the apriltag information
     * as well as the camera identifier.  Uses angle history of the robot
     * to implement new pose as opposed to the angle of the given apriltag
     * 
     * Note:  Apriltag yaw is used to identify unreliable tags and will return
     * null if there is too large a discrepency in angle or if the angle
     * history is insufficient to account for the timestamp given.
     * 
     * @param  aprilX the horizontal translation of the apriltag as viewed
     * from the camera
     * @param aprilZ the distance the apriltag is away from the lens of the
     * camera
     * @param aprilYaw the yaw of the apriltag as observed by the camera
     * @param aprilID the apriltag ID number
     * @param timestamp the time at which the apriltag was observed
     * @param cameraNum the camera on the robot which observed the apriltag
     * 
     * @return the observed pose of the robot, null if a valid
     * one doesn't exist
     * 
     */
    

    private Pose2d constructPose(double aprilX, double aprilZ, double aprilYaw, int aprilID, double timestamp, int cameraNum){
        Pose2d returnPose = null;
        
        //Check if angle history exists
        if(angleInterpolate.getSample(timestamp).isEmpty()) return returnPose;
        Rotation2d robotRotation = angleInterpolate.getSample(timestamp).get();
        Rotation2d aprilPred = aprilTagGlobalPoses[aprilID].getRotation().minus(robotRotation);
        Rotation2d aprilMeas = new Rotation2d(Math.toRadians(aprilYaw));
        //Check if angle from apriltag makes sense
        if(Math.abs(aprilPred.minus(aprilMeas).getDegrees())>PoseConstants.ANGLE_DIFF_TOLERANCE)
            return returnPose;

        //Use angle of what apriltag should be observed as for the transformation
        Transform2d aprilLocal = new Transform2d(new Translation2d(-aprilZ, aprilX), aprilPred);
        returnPose = aprilTagGlobalPoses[aprilID].plus(aprilLocal).plus(VisionConstants.CAMERATRANSFORMS[cameraNum]);

        
        return returnPose;
    }

    // private void constructPose(Pose2d cameraPose, Rotation2d cameraAngle, double timestamp) {
    //     // Setup different poses of apriltags and robots relative to each other
    //     Timer calcTime = new Timer();
    //     calcTime.start();
    //     Pose2d newRobotPose = new Pose2d();

    //     for (int aTagID : aprilTagIDs) {
    //         // aTag2int = (int)aTagIDs[i];

    //         // Create pose of robot with respect to the apriltag;
    //         // robotRelativePose = new Pose2d(-cameraTranslationZ[aTagID],
    //         // -cameraTranslationX[aTagID],
    //         // new Rotation2d(aTagRotation[aTagID]));

    //         // Create rotation of apriltag with respect to the robot

    //         // Negative of Angle that robot must rotate to center the apriltag in vision
    //         // robotRelativeAngle = new Rotation2d(cameraTranslationZ[aTagID],
    //         // cameraTranslationX[aTagID]);

    //         // Create transformation of robot with respect to the apriltag, angle is the
    //         // corrected angle based on apriltag relative pose
    //         // SmartDashboard.putBoolean("Available Sample?",
    //         // angleInterpolate.getSample(timestamp).isPresent());
    //         if (angleInterpolate.getSample(timestamp).isPresent() && confidenceNT[aTagID] > 0 && aTagID > 0) {
    //             // double pigeonAngle = pigeon.getAngle();
    //             // Rotation2d robotRotation = Rotation2d.fromDegrees(pigeonAngle);

    //             Pose2d aprilTagPose = aprilTagGlobalPoses[aTagID];
    //             Pose2d globalToAprilTag = new Pose2d(aprilTagPose.getTranslation(),
    //                     angleInterpolate.getSample(timestamp).get().plus(cameraAngle));

    //             Transform2d aprilTagTransform = new Transform2d(
    //                     new Translation2d(-1.0*zTranslationNT[aTagID], xTranslationNT[aTagID]),
    //                     new Rotation2d());

    //             Transform2d aprilToRobot = new Transform2d(new Pose2d(), cameraPose.plus(aprilTagTransform));
    //             newRobotPose = globalToAprilTag.plus(aprilToRobot);
    //             Rotation2d measuredYaw = aprilTagGlobalPoses[aTagID].getRotation()
    //                     .plus(new Rotation2d(Math.toRadians(yawRotationNT[aTagID])).minus(cameraAngle));

    //             newRobotPose = new Pose2d(newRobotPose.getTranslation(), angleInterpolate.getSample(timestamp).get());
    //             double positionError;
    //             double angleError;
    //             try {
    //                 positionError = VisionConstants.DISTANCEERRORFACTOR * Math.abs(Math.pow(zTranslationNT[aTagID], 2))
    //                         / confidenceNT[aTagID];
    //                 angleError = VisionConstants.ANGLEERRORFACTOR * Math.abs(zTranslationNT[aTagID])
    //                         / confidenceNT[aTagID];
    //             } catch (Exception e) {
    //                 positionError = 1;
    //                 angleError = 10;
    //             }

    //             // System.out.println("Aligning with Apriltag " + aTagID);,
    //             if (newRobotPose.getTranslation().getDistance(new Translation2d()) > 0
    //                     && newRobotPose.getTranslation().getDistance(
    //                             new Translation2d(VisionConstants.MID_FIELD_X_POS,
    //                                     VisionConstants.MID_FIELD_Y_POS)) < 10) {
    //                 // SmartDashboard.putNumber("TimetoCalc", calcTime.get());

    //                 double oldToNewDistance = newRobotPose.getTranslation().getDistance(robotDrive.getPose().getTranslation());
                
    //                 if (oldToNewDistance > VisionConstants.ADD_VISION_TOLERANCE){
    //                     Transform2d oldToNewTransform = newRobotPose.minus(robotDrive.getPose()).div(oldToNewDistance).times(VisionConstants.ADD_VISION_TOLERANCE);
    //                     newRobotPose = robotDrive.getPose().plus(oldToNewTransform);
    //                 } 

    //                 ParallelPoseAdding multithreadingRunnable = new ParallelPoseAdding(newRobotPose, timestamp,
    //                             new MatBuilder<N3, N1>(Nat.N3(), Nat.N1()).fill(positionError, positionError,
    //                                     angleError));

    //                 Thread addThread = new Thread(multithreadingRunnable);
    //                 // offer a new thread to the blocking queue if not full
    //                 if (blockingQueue.offer(addThread))
    //                     addThread.start();
    //                 // Clear the queue of finished threads
    //                 for (int threadNum = 0; blockingQueue.peek() != null
    //                         && !blockingQueue.peek().isAlive()
    //                         && threadNum < VisionConstants.NUM_THREADS; threadNum++) {
    //                     blockingQueue.poll();
    //                 }
    //                 SmartDashboard.putNumber("Active Threads", blockingQueue.size());


    //                 // else if (zTranslationNT[aTagID] < VisionConstants.DISTANCE_RESET_TOLERANCE
    //                 //         && Math.hypot(robotDrive.getOpenLoopSpeeds().vxMetersPerSecond, robotDrive
    //                 //                 .getOpenLoopSpeeds().vyMetersPerSecond) < VisionConstants.SPEED_RESET_TOLERANCE
    //                 //         && robotDrive
    //                 //                 .getOpenLoopSpeeds().omegaRadiansPerSecond < VisionConstants.OMEGA_RESET_TOLERANCE) {
    //                 //     robotDrive.setPose(newRobotPose);
    //                 // }

    //                 // if (!blockingQueue.offer(addThread)){
    //                 // Thread removeThread = blockingQueue.poll();
    //                 // if (removeThread != null){
    //                 // removeThread.interrupt();
    //                 // }
    //                 // blockingQueue.offer(addThread);
    //                 // }
    //                 // robotDrive.addVisionMeasurement(newRobotPose, timestamp,
    //                 // new MatBuilder<N3, N1>(Nat.N3(), Nat.N1()).fill(positionError, positionError,
    //                 // angleError));
    //             }

    //             // SmartDashboard.putNumber("Camera Pose x",
    //             // cameraPose.getTranslation().getX());
    //             // SmartDashboard.putNumber("Camera Pose y",
    //             // cameraPose.getTranslation().getY());
    //             // SmartDashboard.putNumber("Camera Pose theta",
    //             // cameraPose.getRotation().getDegrees());
    //             // SmartDashboard.putNumber("Robot Relative Pose x",
    //             // aprilToRobot.getTranslation().getX());
    //             // SmartDashboard.putNumber("Robot Relative Pose y",
    //             // aprilToRobot.getTranslation().getY());
    //             // SmartDashboard.putNumber("Robot Relative Pose theta",
    //             // aprilToRobot.getRotation().getDegrees());
    //             // SmartDashboard.putNumber("Timestamp addition", timestamp);
    //             // SmartDashboard.putNumber("Distance error", positionError);

    //             // aprilTagRelativeTransformation = new
    //             // Transform2d(robotRelativePose.getTranslation(),
    //             // aprilTagGlobalPoses[aTagID].getRotation().plus(aprilTagYaw)
    //             // .minus(angleInterpolate.getSample(timestamp).get()));

    //             // System.out.println("Calculating Pose: " +
    //             // angleInterpolate.getSample(timestamp));

    //             // newCameraPose = (new Pose2d(aprilTagGlobalPoses[aTagID].getTranslation(),
    //             // angleInterpolate.getSample(timestamp).get())).plus(aprilTagRelativeTransformation);
    //             // newRobotPose = newCameraPose.plus(cameraPose);

    //         }
    //     }
    // }

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

    public Pose2d[] getAprilTagGlobalPoses() {
        return aprilTagGlobalPoses;
    }

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

    // public void updateRobotPose() {
    //     NetworkTable subTable = table.getSubTable("Camera 0");
    //     aprilDetect(subTable);
    //     if (aprilTagIDs.length > 0) {
    //         double pigeonAngle = pigeon.getAngle();
    //         Rotation2d robotRotation = Rotation2d.fromDegrees(pigeonAngle);

    //         Pose2d aprilTagPose = aprilTagGlobalPoses[aprilTagIDs[0]];
    //         Pose2d globalToAprilTag = new Pose2d(aprilTagPose.getTranslation(), robotRotation);

    //         // Pose2d rotationPose = new Pose2d(0, 0, robotRotation);
    //         Transform2d aprilTagTransform = new Transform2d(
    //                 new Translation2d(-zTranslationNT[aprilTagIDs[0]], xTranslationNT[aprilTagIDs[0]]),
    //                 new Rotation2d());
    //         // SmartDashboard.putNumber("transform x",
    //         // globalToAprilTag.getTranslation().getX());
    //         // SmartDashboard.putNumber("transform y",
    //         // globalToAprilTag.getTranslation().getY());
    //         // SmartDashboard.putNumber("transform theta",
    //         // globalToAprilTag.getRotation().getDegrees());
    //         // Pose2d cameraPose = rotationPose.plus(aprilTagTransform);
    //         // Transform2d aprilTagToCamera = new Transform2d(new Pose2d(), cameraPose);

    //         Transform2d cameraToRobot = VisionConstants.CAMERATRANSFORMS[0];

    //         Transform2d aprilToRobot = aprilTagTransform.plus(cameraToRobot);
    //         robotPose = globalToAprilTag.plus(aprilToRobot);

    //         // Pose2d finalPose = new
    //         // Pose2d().transformBy(globalToAprilTag).transformBy(aprilTagToCamera).transformBy(cameraToRobot);
    //         // robotPose = new Pose2d(finalPose.getX(), finalPose.getY(),
    //         // robotDrive.getPose().getRotation());
    //     } else {
    //         robotPose = null;
    //     }
    // }

    public Pose2d getRobotPose() {
        return robotPose;
    }

    /**
     * Checks whether the camera is currently seeing an April Tag
     * 
     * @return whether aprilTagIds = 0
     */
    public boolean noApples() {
        boolean apples = aprilTagIDs.length > 0;
        // lights.apples(apples);
        return !apples;
    }

    public void updateCubeValues(){
        cubeX = getSubValue("X Translation","Camera 2").getDouble(0.0);
        cubeY = getSubValue("Y Translation","Camera 2").getDouble(0.0);
        cubeAngle = getSubValue("Angle","Camera 2").getDouble(0.0);
        cubePercentArea = getSubValue("Percent Area","Camera 2").getDouble(0.0);
    }

    public void resetCubeValues(){
        cubeX = 0;
        cubeY = 0;
        cubeAngle = 0;
        cubePercentArea = 0;
    }

    public double getCubeX() {
        return cubeX;
    }

    public double getCubeY() {
        return cubeY;
    }

    public double getCubeAngle() {
        return cubeAngle;
    }

    public double getCubePercentArea() {
        return cubePercentArea;
    }

    public boolean isCube(){
        return cubePercentArea >= 0.05;
    }
}
