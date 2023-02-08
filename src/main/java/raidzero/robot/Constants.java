package raidzero.robot;

import java.nio.file.Path;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Constants {

        public static final class ArmConstants {
                /** Arm Kinematics Constants */
                public static final double LOWER_ARM_LENGTH = 0.91; // in meters
                public static final double UPPER_ARM_LENGTH = 0.91;

                public static final double BASE_PIVOT_COG = 0.0; // in meters
                public static final double JOINT_COM = 0.0;

                public static final double LOWER_ARM_WEIGHT = 0.0; // in pounds
                public static final double UPPER_ARM_WEIGHT = 0.0;

                public static final double LOWER_ARM_MOI = 0.0;
                public static final double UPPER_ARM_MOI = 0.0;

                public static final int LINKAGES = 2;

                /**
                 * Constants for a DC brushed motor.
                 * nominal_voltage -- voltage at which the motor constants were measured
                 * stall_torque -- current draw when stalled in Newton-meters
                 * stall_current -- current draw when stalled in Amps
                 * free_current -- current draw under no load in Amps
                 * free_speed -- angular velocity under no load in RPM
                 **/
                public static final double STALL_TORQUE = 2.6;
                public static final double STALL_CURRENT = 105;
                public static final double FREE_SPEED = 5676;
                public static final double BASE_PIVOT_GEAR_RATIO = 150;
                public static final double JOINT_GEAR_RATIO = 150;
                public static final double BASE_PIVOT_MOTORS = 2;
                public static final double JOINT_MOTORS = 2;
                public static final double GRAVITY = 9.81;

                /** Regular Constants */
                public static final int LOWER_LEADER_ID = 1;
                public static final int LOWER_FOLLOWER_ID = 0;
                public static final int UPPER_LEADER_ID = 2;
                public static final int UPPER_FOLLOWER_ID = 0;

                public static final boolean LOWER_MOTOR_INVERSION = true;
                public static final boolean UPPER_MOTOR_INVERSION = true;

                public static final int LOWER_CURRENT_LIMIT = 200;
                public static final int UPPER_CURRENT_LIMIT = 200;

                public static final SparkMaxLimitSwitch.Type LOWER_FORWARD_LIMIT_TYPE = SparkMaxLimitSwitch.Type.kNormallyOpen;
                public static final SparkMaxLimitSwitch.Type LOWER_REVERSE_LIMIT_TYPE = SparkMaxLimitSwitch.Type.kNormallyOpen;

                public static final double LOWER_ZERO_OFFSET = 0.0;
                public static final double UPPER_ZERO_OFFSET = 0.0;

                public static final boolean LOWER_ENCODER_INVERSION = false;
                public static final boolean UPPER_ENCODER_INVERSION = false;

                public static final int LOWER_SMART_MOTION_SLOT = 0;
                public static final int UPPER_SMART_MOTION_SLOT = 0;

                public static final double TICKS_TO_DEGREES = 3.0;

                public static final double LOWER_KF = 0.000166;
                public static final double LOWER_KP = 0.000187;
                public static final double LOWER_KI = 0.0;
                public static final double LOWER_KD = 0.000090;
                public static final double LOWER_MIN_VEL = 0.0;
                public static final double LOWER_MAX_VEL = 3760; 
                public static final double LOWER_MAX_ACCEL = 1960;
                public static final double LOWER_MIN_ERROR = 0.0;
                
                public static final double UPPER_KF = 0.000166;
                public static final double UPPER_KP = 0.000106;
                public static final double UPPER_KI = 0.0;
                public static final double UPPER_KD = 0.000009;
                public static final double UPPER_MIN_VEL = 0.0;
                public static final double UPPER_MAX_VEL = 3760;
                public static final double UPPER_MAX_ACCEL = 1960;
                public static final double UPPER_MIN_ERROR = 0.0;

                // public static final double LOWER_KF = 0.000166;
                // public static final double LOWER_KP = 0.000187;
                // public static final double LOWER_KI = 0.0;
                // public static final double LOWER_KD = 0.000090;
                // public static final double LOWER_MIN_VEL = 0.0;
                // public static final double LOWER_MAX_VEL = 500;
                // public static final double LOWER_MAX_ACCEL = 250;
                // public static final double LOWER_MIN_ERROR = 0.0;
                
                // public static final double UPPER_KF = 0.000166;
                // public static final double UPPER_KP = 0.000106;
                // public static final double UPPER_KI = 0.0;
                // public static final double UPPER_KD = 0.000009;
                // public static final double UPPER_MIN_VEL = 0.0;
                // public static final double UPPER_MAX_VEL = 500;
                // public static final double UPPER_MAX_ACCEL = 250;
                // public static final double UPPER_MIN_ERROR = 0.0;

                public static final double PID_WRAPPING_MIN = 0.0;
                public static final double PID_WRAPPING_MAX = 360.0;
        }

        public static final class WristConstants {
                public static final int ID = 0;

                public static final boolean INVERSION = false;

                public static final int CURRENT_LIMIT = 25;

                public static final boolean ENCODER_INVERSION = false;
                // 1:75 ratio, in degrees
                public static final double POSITION_CONVERSION_FACTOR = 1 / 75 * 360;

                public static final int SMART_MOTION_SLOT = 0;

                public static final double KF = 0.0;
                public static final double KP = 0.0;
                public static final double KI = 0.0;
                public static final double KD = 0.0;

                public static final double MIN_VEL = 0.0;
                public static final double MAX_VEL = 0.0;
                public static final double MAX_ACCEL = 0.0;
                public static final double MIN_ERROR = 0.0;

                public static final double PID_WRAPPING_MIN = 0.0;
                public static final double PID_WRAPPING_MAX = 360.0;
        }

        public static final class TOFSensorConstants {
                public static final int SENSOR_ID = 0;
        }

        public static final double JOYSTICK_DEADBAND = 0.07;
        public static final int TIMEOUT_MS = 20;
        public static final double TIMEOUT_S = TIMEOUT_MS / 1000.0f;
        public static final int SECONDS_IN_MINUTE = 60;
        public static final double SQRTTWO = Math.sqrt(2);
        public static final String CANBUS_STRING = "seCANdary";
        public static final PneumaticsModuleType PNEUMATICS_MODULE_TYPE = PneumaticsModuleType.REVPH;
        public static final double VOLTAGE_COMP = 12.0;
}
