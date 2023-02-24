package raidzero.robot.submodules;

import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import raidzero.robot.Constants;
import raidzero.robot.Constants.SwerveConstants;
import raidzero.robot.Constants.VisionConstants;
import raidzero.robot.Constants.DriveConstants;
import raidzero.robot.dashboard.Tab;

public class Swerve extends Submodule {

    private enum ControlState {
        OPEN_LOOP, PATHING, AUTO_AIM
    };

    // Auto-aim target Locations
    public enum AutoAimLocation {
        BLL, BLM, BLR, BML, BMM, BMR, BRL, BRM, BRR,
        RLL, RLM, RLR, RML, RMM, RMR, RRL, RRM, RRR,
        B_LOAD,
        R_LOAD
    };

    private class WPI_Pigeon2_Helper extends WPI_Pigeon2 {
        public WPI_Pigeon2_Helper(int deviceNumber, String canbus) {
            super(deviceNumber, canbus);
        }

        public double getAngle() {
            return -super.getAngle();
        }
    }

    private static Swerve instance = null;

    public static Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }
        return instance;
    }

    private Swerve() {
    }

    private static final Vision vision = Vision.getInstance();

    private SwerveModule topRightModule = new SwerveModule();
    private SwerveModule topLeftModule = new SwerveModule();
    private SwerveModule bottomLeftModule = new SwerveModule();
    private SwerveModule bottomRightModule = new SwerveModule();

    private WPI_Pigeon2_Helper pigeon;

    private SwerveDrivePoseEstimator odometry;

    private Pose2d currentPose;
    private Pose2d prevPose;
    private Field2d fieldPose = new Field2d();

    private PathPlannerTrajectory currentTrajectory;
    private boolean firstPath = true;
    private boolean overLimit = false;
    // private Rotation2d targetAngle;
    private PIDController xController, yController, thetaController;
    private Timer timer = new Timer();

    private Pose2d desiredAutoAimPose;
    private PIDController autoAimXController, autoAimYController, autoAimThetaController;

    private ControlState controlState = ControlState.OPEN_LOOP;

    public void onStart(double timestamp) {
        controlState = ControlState.OPEN_LOOP;

        zero();
        firstPath = true;
    }

    public void onInit() {
        pigeon = new WPI_Pigeon2_Helper(SwerveConstants.IMU_ID, Constants.CANBUS_STRING);
        Shuffleboard.getTab(Tab.MAIN).add("Pigey", pigeon).withSize(2, 2).withPosition(4, 4);

        topLeftModule.onInit(
                SwerveConstants.FRONT_LEFT_THROTTLE_ID,
                SwerveConstants.FRONT_LEFT_ROTOR_ID,
                SwerveConstants.FRONT_LEFT_ENCODER_ID,
                SwerveConstants.FRONT_LEFT_ROTOR_OFFSET);
        topRightModule.onInit(
                SwerveConstants.FRONT_RIGHT_THROTTLE_ID,
                SwerveConstants.FRONT_RIGHT_ROTOR_ID,
                SwerveConstants.FRONT_RIGHT_ENCODER_ID,
                SwerveConstants.FRONT_RIGHT_ROTOR_OFFSET);
        bottomLeftModule.onInit(
                SwerveConstants.REAR_LEFT_THROTTLE_ID,
                SwerveConstants.REAR_LEFT_ROTOR_ID,
                SwerveConstants.REAR_LEFT_ENCODER_ID,
                SwerveConstants.REAR_LEFT_ROTOR_OFFSET);
        bottomRightModule.onInit(
                SwerveConstants.REAR_RIGHT_THROTTLE_ID,
                SwerveConstants.REAR_RIGHT_ROTOR_ID,
                SwerveConstants.REAR_RIGHT_ENCODER_ID,
                SwerveConstants.REAR_RIGHT_ROTOR_OFFSET);

        odometry = new SwerveDrivePoseEstimator(
                SwerveConstants.KINEMATICS,
                Rotation2d.fromDegrees(pigeon.getAngle()),
                getModulePositions(),
                DriveConstants.STARTING_POSE,
                DriveConstants.STATE_STDEVS_MATRIX,
                DriveConstants.VISION_STDEVS_MATRIX);

        xController = new PIDController(SwerveConstants.XCONTROLLER_KP, 0, 0);
        yController = new PIDController(SwerveConstants.YCONTROLLER_KP, 0, 0);
        thetaController = new PIDController(SwerveConstants.THETACONTROLLER_KP, 0, SwerveConstants.THETACONTROLLER_KD);
        xController.setTolerance(SwerveConstants.XCONTROLLER_TOLERANCE);
        yController.setTolerance(SwerveConstants.YCONTROLLER_TOLERANCE);
        thetaController.setTolerance(SwerveConstants.THETACONTROLLER_TOLERANCE);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        autoAimXController = new PIDController(SwerveConstants.AA_XCONTROLLER_KP, 0, 0);
        autoAimYController = new PIDController(SwerveConstants.AA_YCONTROLLER_KP, 0.0, 0.0);
        autoAimThetaController = new PIDController(SwerveConstants.AA_THETACONTROLLER_KP, 0,
                SwerveConstants.THETACONTROLLER_KD);
        autoAimXController.setTolerance(SwerveConstants.AA_XCONTROLLER_TOLERANCE);
        autoAimYController.setTolerance(SwerveConstants.AA_YCONTROLLER_TOLERANCE);
        autoAimThetaController.setTolerance(SwerveConstants.AA_THETACONTROLLER_TOLERANCE);
        autoAimThetaController.enableContinuousInput(-Math.PI, Math.PI);

        zero();
        prevPose = new Pose2d();

        PathPlannerServer.startServer(5811);
    }

    @Override
    public void update(double timestamp) {
        if (controlState == ControlState.PATHING) {
            updatePathing();
        }
        // else if (controlState == ControlState.AUTO_AIM) {
        // updateAutoAim();
        // }
        topRightModule.update(timestamp);
        topLeftModule.update(timestamp);
        bottomLeftModule.update(timestamp);
        bottomRightModule.update(timestamp);

        prevPose = currentPose;
        currentPose = updateOdometry();
        fieldPose.setRobotPose(currentPose);

        // This needs to be moved somewhere else.....
        SmartDashboard.putData(fieldPose);

        SmartDashboard.putNumber("X pose", odometry.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Y pose", odometry.getEstimatedPosition().getY());
        SmartDashboard.putNumber("Theta pose", odometry.getEstimatedPosition().getRotation().getDegrees());

        checkThrottleSpeed();
    }

    /**
     * Runs components in the submodule that have continuously changing inputs.
     */
    @Override
    public void run() {
        topRightModule.run();
        topLeftModule.run();
        bottomLeftModule.run();
        bottomRightModule.run();
    }

    @Override
    public void stop() {
        controlState = ControlState.OPEN_LOOP;
        topRightModule.stop();
        topLeftModule.stop();
        bottomLeftModule.stop();
        bottomRightModule.stop();
    }

    /**
     * Resets the sensor(s) to zero.
     */
    @Override
    public void zero() {
        zeroHeading();
        setPose(new Pose2d());
        topRightModule.zero();
        topLeftModule.zero();
        bottomLeftModule.zero();
        bottomRightModule.zero();
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                topLeftModule.getModulePosition(),
                topRightModule.getModulePosition(),
                bottomLeftModule.getModulePosition(),
                bottomRightModule.getModulePosition()
        };
    }

    /**
     * Checks the Speed of the throttle and updates the overlimit boolean
     */
    public void checkThrottleSpeed() {
        if (topLeftModule.getThrottlePercentSpeed() > 0.3 || topRightModule.getThrottlePercentSpeed() > 0.3
                || bottomLeftModule.getThrottlePercentSpeed() > 0.3
                || bottomRightModule.getThrottlePercentSpeed() > 0.3)
            overLimit = true;
        else
            overLimit = false;
    }

    /**
     * Checks whether the swerve is over it's safe speed limit
     * 
     * @return is over limit?
     */
    public boolean isOverLimit() {
        return overLimit;
    }

    /**
     * Zeroes the heading of the swerve.
     */
    public void zeroHeading() {
        pigeon.setYaw(0, Constants.TIMEOUT_MS);
    }

    public void setPose(Pose2d pose) {
        odometry.resetPosition(Rotation2d.fromDegrees(pigeon.getAngle()), getModulePositions(), pose);
    }

    public Pose2d getPose() {
        return odometry.getEstimatedPosition();
    }

    public SwerveDrivePoseEstimator getPoseEstimator() {
        return odometry;
    }

    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        try {
            // visionMeasurementStdDevs = new MatBuilder<N3, N1>(Nat.N3(),
            // Nat.N1()).fill(0.2, 0.2, 0.1);
            // odometry.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
            // odometry.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds,
            // visionMeasurementStdDevs);
        } catch (Exception e) {
            System.out.println("Cholesky decomposition failed, reverting...:");
            // pigeon.setYaw(visionRobotPoseMeters.getRotation().getDegrees());
            // setPose(visionRobotPoseMeters);
        }
    }

    public Pose2d getPrevPose() {
        return prevPose;
    }

    /**
     * Updates odometry
     * 
     * @return current position
     */
    private Pose2d updateOdometry() {
        try {
            return odometry.update(
                    Rotation2d.fromDegrees(pigeon.getAngle()),
                    getModulePositions());
        } catch (Exception e) {
            System.out.println(e);
            return odometry.getEstimatedPosition();
        }
    }

    /**
     * Drives robot (primarily used for teleop manual control)
     * 
     * @param xSpeed        speed in x direction
     * @param ySpeed        speed in y direction
     * @param angularSpeed  turn speed
     * @param fieldOriented
     */
    public void drive(double xSpeed, double ySpeed, double angularSpeed, boolean fieldOriented) {
        controlState = ControlState.OPEN_LOOP;
        boolean ignoreAngle = false;
        if (Math.abs(xSpeed) < 0.1 && Math.abs(ySpeed) < 0.1 && Math.abs(angularSpeed) < 0.1) {
            ignoreAngle = true;
        }
        var targetState = SwerveConstants.KINEMATICS.toSwerveModuleStates(
                fieldOriented
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                xSpeed, ySpeed, angularSpeed,
                                Rotation2d.fromDegrees(pigeon.getAngle()))
                        : new ChassisSpeeds(xSpeed, ySpeed, angularSpeed));
        SwerveDriveKinematics.desaturateWheelSpeeds(targetState, 1);
        topLeftModule.setTargetState(targetState[0], ignoreAngle, true, true);
        topRightModule.setTargetState(targetState[1], ignoreAngle, true, true);
        bottomLeftModule.setTargetState(targetState[2], ignoreAngle, true, true);
        bottomRightModule.setTargetState(targetState[3], ignoreAngle, true, true);
    }

    /**
     * Follow path
     * 
     * @param trajectory desired path
     */
    public void followPath(PathPlannerTrajectory trajectory) {
        if (controlState == ControlState.PATHING) {
            return;
        }
        if (firstPath) {
            setPose(trajectory.getInitialHolonomicPose());
            firstPath = false;
        }
        controlState = ControlState.PATHING;
        currentTrajectory = trajectory;

        timer.reset();
        timer.start();
    }

    /**
     * A better updatePathing(), featuring:
     * - actually allows the robot to turn
     * - actually reads turn changes from the pathplanner trajectory
     * - fully feedback, no more weird feedforward stuff that doesnt actually work
     */
    private void updatePathing() {
        PathPlannerState state = (PathPlannerState) currentTrajectory.sample(timer.get());
        double xSpeed = xController.calculate(getPose().getX(), state.poseMeters.getX());
        double ySpeed = yController.calculate(getPose().getY(), state.poseMeters.getY());
        double thetaSpeed = thetaController.calculate(getPose().getRotation().getRadians(),
                state.holonomicRotation.getRadians());
        // Math
        SmartDashboard.putNumber("theta speed", thetaSpeed);

        ChassisSpeeds desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed,
                ySpeed,
                thetaSpeed,
                getPose().getRotation());
        PathPlannerServer.sendPathFollowingData(state.poseMeters, getPose());

        SwerveModuleState[] desiredState = SwerveConstants.KINEMATICS.toSwerveModuleStates(desiredSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredState, 1);
        topLeftModule.setTargetState(desiredState[0], false, true, true);
        topRightModule.setTargetState(desiredState[1], false, true, true);
        bottomLeftModule.setTargetState(desiredState[2], false, true, true);
        bottomRightModule.setTargetState(desiredState[3], false, true, true);
    }

    /**
     * Get total path time
     * 
     * @return path time
     */
    public double getPathingTime() {
        return timer.get();
    }

    /**
     * Check if robot has finished pathing
     * 
     * @return robot pathing state
     */
    public boolean isFinishedPathing() {
        if (xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint()) {
            if (timer.hasElapsed(currentTrajectory.getTotalTimeSeconds())) {
                return true;
            }
        }
        return false;
    }

    private AutoAimLocation prevAutoAimLocation;

    /**
     * Auto aim robot to desired location
     * 
     * @param location auto aim location
     */
    public void autoAim(AutoAimLocation location) {
        controlState = ControlState.AUTO_AIM;
        if (prevAutoAimLocation != location) {
            setPose(new Pose2d(vision.getRobotPose().getX(), vision.getRobotPose().getY(),
                    Rotation2d.fromDegrees(pigeon.getAngle())));
        }

        prevAutoAimLocation = location;
        // zero();
        switch (location) {
            /**
             * Blue Alliance
             */
            case BLL:
                desiredAutoAimPose = new Pose2d(VisionConstants.BLL[0], VisionConstants.BLL[1],
                        Rotation2d.fromDegrees(VisionConstants.BLL[2]));
                System.out.println("bll");
                break;
            case BLM:
                desiredAutoAimPose = new Pose2d(VisionConstants.BLM[0], VisionConstants.BLM[1],
                        Rotation2d.fromDegrees(VisionConstants.BLM[2]));
                System.out.println("blm");
                break;
            case BLR:
                desiredAutoAimPose = new Pose2d(VisionConstants.BLR[0], VisionConstants.BLR[1],
                        Rotation2d.fromDegrees(VisionConstants.BLR[2]));
                System.out.println("blr");
                break;

            case BML:
                desiredAutoAimPose = new Pose2d(VisionConstants.BML[0], VisionConstants.BML[1],
                        Rotation2d.fromDegrees(VisionConstants.BML[2]));
                System.out.println("bml");
                break;
            case BMM:
                desiredAutoAimPose = new Pose2d(VisionConstants.BMM[0], VisionConstants.BMM[1],
                        Rotation2d.fromDegrees(VisionConstants.BMM[2]));
                System.out.println("bmm");
                break;
            case BMR:
                desiredAutoAimPose = new Pose2d(VisionConstants.BMR[0], VisionConstants.BMR[1],
                        Rotation2d.fromDegrees(VisionConstants.BMR[2]));
                System.out.println("bmr");
                break;

            case BRL:
                desiredAutoAimPose = new Pose2d(VisionConstants.BRL[0], VisionConstants.BRL[1],
                        Rotation2d.fromDegrees(VisionConstants.BRL[2]));
                System.out.println("brl");
                break;
            case BRM:
                desiredAutoAimPose = new Pose2d(VisionConstants.BRM[0], VisionConstants.BRM[1],
                        Rotation2d.fromDegrees(VisionConstants.BRM[2]));
                System.out.println("brm");
                break;
            case BRR:
                desiredAutoAimPose = new Pose2d(VisionConstants.BRR[0], VisionConstants.BRR[1],
                        Rotation2d.fromDegrees(VisionConstants.BRR[2]));
                System.out.println("brr");
                break;
            /**
             * Red Alliance
             */
            case RLL:
                desiredAutoAimPose = new Pose2d(VisionConstants.RLL[0], VisionConstants.RLL[1],
                        Rotation2d.fromDegrees(VisionConstants.RLL[2]));
                System.out.println("rll");
                break;
            /**
             * Red Alliance
             */
            case RLM:
                desiredAutoAimPose = new Pose2d(VisionConstants.RLM[0], VisionConstants.RLM[1],
                        Rotation2d.fromDegrees(VisionConstants.RLM[2]));
                System.out.println("rlm");
                break;
            case RLR:
                desiredAutoAimPose = new Pose2d(VisionConstants.RLR[0], VisionConstants.RLR[1],
                        Rotation2d.fromDegrees(VisionConstants.RLR[2]));
                System.out.println("rlr");
                break;
            case RML:
                desiredAutoAimPose = new Pose2d(VisionConstants.RML[0], VisionConstants.RML[1],
                        Rotation2d.fromDegrees(VisionConstants.RML[2]));
                System.out.println("rml");
                break;
            case RMM:
                desiredAutoAimPose = new Pose2d(VisionConstants.RMM[0], VisionConstants.RMM[1],
                        Rotation2d.fromDegrees(VisionConstants.RMM[2]));
                System.out.println("rmm");
                break;
            case RMR:
                desiredAutoAimPose = new Pose2d(VisionConstants.RMR[0], VisionConstants.RMR[1],
                        Rotation2d.fromDegrees(VisionConstants.RMR[2]));
                System.out.println("rmr");
                break;
            case RRL:
                desiredAutoAimPose = new Pose2d(VisionConstants.RRL[0], VisionConstants.RRL[1],
                        Rotation2d.fromDegrees(VisionConstants.RRL[2]));
                System.out.println("rrl");
                break;
            case RRM:
                desiredAutoAimPose = new Pose2d(VisionConstants.RRM[0], VisionConstants.RRM[1],
                        Rotation2d.fromDegrees(VisionConstants.RRM[2]));
                System.out.println("rrm");
                break;
            case RRR:
                desiredAutoAimPose = new Pose2d(VisionConstants.RRR[0], VisionConstants.RRR[1],
                        Rotation2d.fromDegrees(VisionConstants.RRR[2]));
                System.out.println("rrr");
                break;
            case B_LOAD:
                desiredAutoAimPose = null;
                break;
            case R_LOAD:
                desiredAutoAimPose = null;
                break;
            default:
                break;
        }

        updateAutoAim();
    }

    /**
     * Update Auto Aim
     */
    public void updateAutoAim() {
        double xSpeed = autoAimXController.calculate(getPose().getX(), desiredAutoAimPose.getX());
        double ySpeed = autoAimYController.calculate(getPose().getY(), desiredAutoAimPose.getY());
        double thetaSpeed = autoAimThetaController.calculate(getPose().getRotation().getRadians(),
                desiredAutoAimPose.getRotation().getRadians());

        SmartDashboard.putNumber("x speed", xSpeed);
        SmartDashboard.putNumber("y speed", ySpeed);

        ChassisSpeeds desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed,
                ySpeed,
                thetaSpeed,
                getPose().getRotation());
        SwerveModuleState[] desiredState = SwerveConstants.KINEMATICS.toSwerveModuleStates(desiredSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredState, 0.65);
        topLeftModule.setTargetState(desiredState[0], false, true, true);
        topRightModule.setTargetState(desiredState[1], false, true, true);
        bottomLeftModule.setTargetState(desiredState[2], false, true, true);
        bottomRightModule.setTargetState(desiredState[3], false, true, true);
    }

    /**
     * Test swerve modules
     * 
     * @param quadrant       module quadrant [I, II, III, IV]
     * @param throttleOutput output of throttle motor
     * @param rotorOutput    output of rotor motor
     */
    public void testModule(int quadrant, double throttleOutput, double rotorOutput) {
        if (quadrant == 1) {
            topLeftModule.testThrottleAndRotor(throttleOutput, rotorOutput);
        } else if (quadrant == 2) {
            bottomLeftModule.testThrottleAndRotor(throttleOutput, rotorOutput);
        } else if (quadrant == 3) {
            bottomRightModule.testThrottleAndRotor(throttleOutput, rotorOutput);
        } else {
            topRightModule.testThrottleAndRotor(throttleOutput, rotorOutput);
        }
    }

    /**
     * Set drive throttle ramp rate
     * 
     * @param val ramp rate (seconds to full speed)
     */
    public void setThrottleRampRate(double val) {
        topRightModule.setRotorRampRate(val);
        topLeftModule.setRotorRampRate(val);
        bottomRightModule.setRotorRampRate(val);
        bottomLeftModule.setRotorRampRate(val);
    }
}