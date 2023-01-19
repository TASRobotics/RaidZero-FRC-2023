package raidzero.robot;

import java.nio.file.Path;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.fasterxml.jackson.annotation.JacksonAnnotationValue;
import com.revrobotics.CANSparkMax.IdleMode;

import org.apache.commons.math3.util.FastMath;
import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import raidzero.robot.utils.InterpolatingDouble;
import raidzero.robot.utils.InterpolatingTreeMap;
import raidzero.robot.utils.MathTools;

public class Constants {
    /**
     * Swerve Constants
     */
    public static final class SwerveConstants {
        public static final int MODULE_ID_TOP_RIGHT = 1;
        public static final int MODULE_ID_TOP_LEFT = 3;
        public static final int MODULE_ID_BOTTOM_LEFT = 5;
        public static final int MODULE_ID_BOTTOM_RIGHT = 7;
        public static final double[] INIT_MODULES_DEGREES = new double[] {
            (130.781 + 90) % 360.0, 
            (190.371 + 90) % 360.0, 
            (125.420 + 90) % 360.0, 
            (311.924 + 90) % 360.0
        };

        // (349.893 + 90) % 360.0, 
        // (149.854 + 90) % 360.0, 
        // (6.416 + 90) % 360.0, 
        // (182.637 + 90) % 360.0
        // Ugly Bot
        // (195.820 + 90) % 360.0, 
        // (278.789 + 90) % 360.0, 
        // (180.000 + 90) % 360.0, 
        // (107.139 + 90) % 360.0

        //new double[] {(57.832 + 0) % 360.0, (205.576 + 0) % 360.0, (212.520 + 0) % 360.0, (308.232 + 0) % 360.0};

        // Robot dimensions
        public static final double ROBOT_WIDTH_INCHES = 28.0; // 23.0 inches on ugly bot
        public static final double ROBOT_HALF_WIDTH_METERS = Units.inchesToMeters(ROBOT_WIDTH_INCHES) / 2.0;
        public static final double ROBOT_RADIUS_INCHES = FastMath.hypot(ROBOT_WIDTH_INCHES, ROBOT_WIDTH_INCHES) / 2.0;
        public static final double ROBOT_RADIUS_METERS = Units.inchesToMeters(ROBOT_RADIUS_INCHES);
        public static final double WHEEL_DIAMETER_INCHES = 4.0;
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES);

        public static final Translation2d MODULE_TOP_LEFT_POSITION = new Translation2d(ROBOT_HALF_WIDTH_METERS, ROBOT_HALF_WIDTH_METERS);
        public static final Translation2d MODULE_TOP_RIGHT_POSITION = new Translation2d(ROBOT_HALF_WIDTH_METERS, -ROBOT_HALF_WIDTH_METERS);
        public static final Translation2d MODULE_BOTTOM_LEFT_POSITION = new Translation2d(-ROBOT_HALF_WIDTH_METERS, ROBOT_HALF_WIDTH_METERS);
        public static final Translation2d MODULE_BOTTOM_RIGHT_POSITION = new Translation2d(-ROBOT_HALF_WIDTH_METERS, -ROBOT_HALF_WIDTH_METERS);

        public static final double FALCON_TICKS = 2048.0;

        // Using SDS 6.75 ratio
        public static final double MOTOR_RATIO = 6.75;
        public static final double MOTOR_TICKS_TO_METERS = Math.PI * WHEEL_DIAMETER_METERS / (FALCON_TICKS * MOTOR_RATIO);

        public static final double CANCODER_TO_DEGREES = 360.0 / 4096.0;

        public static final double MAX_SPEED_MPS = 5.0;  //was 4.0
        public static final double MAX_ANGULAR_SPEED_RPS = 2 * Math.PI;

        public static final boolean MOTOR_INVERSION = false;
        public static final boolean ROTOR_INVERSION = true;
        public static final boolean ROTOR_INVERT_SENSOR_PHASE = true;

        // PID constants
        public static final int PID_PRIMARY_SLOT = 0;
        public static final int PID_AUX_SLOT = 1;

        public static final double MOTOR_MAX_VELOCITY_TICKS_PER_100MS = 21397.0;
        public static final double MOTOR_MAX_VELOCITY_EFFECTIVE_MPS = 3.0;
        public static final double MOTOR_KF = 1.0 * 1023 / MOTOR_MAX_VELOCITY_TICKS_PER_100MS;
        public static final double MOTOR_KP = 0.08;
        public static final double MOTOR_KD = 0.2;

        public static final double ROTOR_MAX_VELOCITY_TICKS_PER_100MS = 3600.0;
        public static final double ROTOR_KF = 1.0 * 1023 / ROTOR_MAX_VELOCITY_TICKS_PER_100MS;
        public static final double ROTOR_KP = 0.8; //1.35
        public static final double ROTOR_KD = 3.0; //0.2
        public static final double ROTOR_TARG_VELO = 1.0 * ROTOR_MAX_VELOCITY_TICKS_PER_100MS;
        public static final double ROTOR_TARG_ACCEL = 10 * ROTOR_TARG_VELO;

        public static final SupplyCurrentLimitConfiguration ROTOR_CURRENT_LIMIT = 
            new SupplyCurrentLimitConfiguration(true, 25, 40, 0.1);
        public static final SupplyCurrentLimitConfiguration THROTTLE_CURRENT_LIMIT = 
            new SupplyCurrentLimitConfiguration(true, 35, 60, 0.1);
    }

    public static final class DriveConstants{
        public static final Rotation2d STARTING_ROTATION = new Rotation2d(0.0);
        public static final Pose2d STARTING_POSE = new Pose2d(0.5,3.0, STARTING_ROTATION);
        private static final double MAX_ACCEL_DISTANCE = 6.0*Math.pow(TIMEOUT_S,2);
        private static final double GYRO_ERROR_DEGREES_TIMEOUT = (0.4/SECONDS_IN_MINUTE)*TIMEOUT_S;
        public static final double CONFIDENCE_TO_ERROR = 1.0;
        public static final Matrix<N3,N1> STATE_STDEVS_MATRIX = new MatBuilder<N3,N1>(Nat.N3(),Nat.N1()).fill(
            MAX_ACCEL_DISTANCE,MAX_ACCEL_DISTANCE, MAX_ACCEL_DISTANCE/SwerveConstants.ROBOT_RADIUS_METERS);
        public static final Matrix<N1,N1> ANGLE_STDEVS_MATRIX = new MatBuilder<N1,N1>(Nat.N1(),Nat.N1()).fill(GYRO_ERROR_DEGREES_TIMEOUT);
        public static final Matrix<N3,N1> VISION_STDEVS_MATRIX = new MatBuilder<N3,N1>(Nat.N3(),Nat.N1()).fill(1.0,1.0,1.0);
    }

    public class PathingConstants {
        public static final int BASE_TRAJ_PERIOD_MS = 0;
        public static final int MIN_POINTS_IN_TALON = 10;
        public static final int TRANSMIT_PERIOD_MS = 20;
    }

    public static final class IntakeConstants {}

    

    public static final class VisionConstants{
        public static final String NAME = "SmartDashboard";
        public static final String APRILTAGFAMILY = "tag36h11";
        private static final String APRILTAGFILENAME = "AprilTagPoses.json";
        public static final Path APRILTAGPATH = Filesystem.getDeployDirectory().toPath().resolve(APRILTAGFILENAME);
        private static final double[][] CAMERALOCATIONS = {{1,2,3},{3,4,5}};
        private static final double[] CAMERAANGLES = {0,180};
        public static final double ANGLEHISTSECS = 2.0;
        public static final double DISTANCETOLERANCE = 3.0;
        //public static final Pose2d[] APRILTAG_POSE2DS = {new Pose2d(1, 1, new Rotation2d(.5))};
        // public final Pose2d[] APRILTAG_POSE2DS = JSONTools.GenerateAprilTagPoses(APRILTAGPATH);
        Path trajectoryFilePath = Filesystem.getDeployDirectory().toPath().resolve("paths/");
    }

    public static final double JOYSTICK_DEADBAND = 0.07;
    public static final int TIMEOUT_MS = 20;
    public static final double TIMEOUT_S = TIMEOUT_MS/1000.0f;
    public static final int SECONDS_IN_MINUTE = 60;
    public static final double SQRTTWO = Math.sqrt(2);
    public static final String CANBUS_STRING = "seCANdary";
    public static final PneumaticsModuleType PNEUMATICS_MODULE_TYPE = PneumaticsModuleType.REVPH;
}
