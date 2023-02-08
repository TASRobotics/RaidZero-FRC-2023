package raidzero.robot.submodules;

import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import raidzero.robot.Constants;
import raidzero.robot.submodules.DoubleJointedArm;
import raidzero.robot.Constants.ArmConstants;
import raidzero.robot.wrappers.LazyCANSparkMax;

public class Arm extends Submodule {

    private DoubleJointedArm Controller = new DoubleJointedArm();

    private ControlState mControlState = ControlState.OPEN_LOOP;
    private double outputOpenLoop = 0.0;

    private double mLowerPercentOut = 0.0;
    private double mUpperPercentOut = 0.0;
    private double mLowerDesiredPosition = 0.0;
    private double mUpperDesiredPosition = 0.0;

    // State of Proximal and Distral Links
    private Pose2d[] state;

    /* Arm Control Constants */
    private double radius = 0;
    private double radius_sq = 0;
    private double theta = 0;
    private double acosarg = 0;
    private double elbow_supplement = 0;
    private double alpha = 0;
    private double[] s1 = { 0, 0 };
    private double[] s2 = { 0, 0 };
    private double[] sf = { 0, 0 };

    private Arm() {
        int numLinkages = ArmConstants.LINKAGES;
        state = new Pose2d[numLinkages];
    }

    private static Arm instance = null;

    public static Arm getInstance() {
        if (instance == null) {
            instance = new Arm();
        }
        return instance;
    }

    private enum ControlState {
        OPEN_LOOP, CLOSED_LOOP
    }

    private final LazyCANSparkMax mLowerLeader = new LazyCANSparkMax(ArmConstants.LOWER_LEADER_ID,
            MotorType.kBrushless);
    // private final LazyCANSparkMax mLowerFollower = new
    // LazyCANSparkMax(ArmConstants.LOWER_FOLLOWER_ID,
    // MotorType.kBrushless);
    private final LazyCANSparkMax mUpperLeader = new LazyCANSparkMax(ArmConstants.UPPER_LEADER_ID,
            MotorType.kBrushless);
    // private final LazyCANSparkMax mUpperFollower = new
    // LazyCANSparkMax(ArmConstants.UPPER_FOLLOWER_ID,
    // MotorType.kBrushless);

    // private final SparkMaxLimitSwitch mLowerForwardLimitSwitch = mLowerLeader
    // .getForwardLimitSwitch(ArmConstants.LOWER_FORWARD_LIMIT_TYPE);
    // private final SparkMaxLimitSwitch mLowerReverseLimitSwitch = mLowerLeader
    // .getReverseLimitSwitch(ArmConstants.LOWER_REVERSE_LIMIT_TYPE);

    // private final SparkMaxAbsoluteEncoder AbsoluteEncoder = mLowerLeader
    // .getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    // private final SparkMaxAbsoluteEncoder mUpperEncoder = mUpperLeader
    // .getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

    private final RelativeEncoder mLowerEncoder = mLowerLeader.getEncoder();
    private final RelativeEncoder mUpperEncoder = mUpperLeader.getEncoder();

    private final SparkMaxPIDController mLowerPIDController = mLowerLeader.getPIDController();
    private final SparkMaxPIDController mUpperPIDController = mUpperLeader.getPIDController();

    @Override
    public void onInit() {
        mLowerLeader.restoreFactoryDefaults();
        // mLowerFollower.restoreFactoryDefaults();
        mUpperLeader.restoreFactoryDefaults();
        // mUpperFollower.restoreFactoryDefaults();

        // mLowerFollower.follow(mLowerLeader, false);
        // mUpperFollower.follow(mUpperLeader, false);

        configLowerSparkMax();
        configUpperSparkMax();
    }

    @Override
    public void onStart(double timestamp) {
    }

    @Override
    public void update(double timestamp) 
        //add forward kinematics here
        Rotation2d[] q = {Rotation2d.fromDegrees(90 - mLowerEncoder.getPosition() * ArmConstants.TICKS_TO_DEGREES), Rotation2d.fromDegrees(-1 * mUpperEncoder.getPosition() * ArmConstants.TICKS_TO_DEGREES)};
        state[0] = new Pose2d(1,2, q[0]);
        state[1] = new Pose2d(1,2, q[1]);

        // SmartDashboard.putNumber("Absolute Angle", AbsoluteEncoder.getPosition());
        SmartDashboard.putNumber("Proximal Angle", state[0].getRotation().getDegrees());
        SmartDashboard.putNumber("Distral Angle", state[1].getRotation().getDegrees());
        SmartDashboard.putNumber("Proximal X ", state[0].getX());
        SmartDashboard.putNumber("Proximal Y ", state[0].getY());
        SmartDashboard.putNumber("Distral X", state[1].getX());
        SmartDashboard.putNumber("Distral Y", state[1].getY());
    }

    @Override
    public void run() {
        if (mControlState == ControlState.OPEN_LOOP) {
            mLowerLeader.set(mLowerPercentOut);
            mUpperLeader.set(mUpperPercentOut);
        } else if (mControlState == ControlState.CLOSED_LOOP) {
            mLowerPIDController.setReference(
                    mLowerDesiredPosition,
                    ControlType.kSmartMotion,
                    ArmConstants.LOWER_SMART_MOTION_SLOT,
                    0,
                    ArbFFUnits.kPercentOut);
            mUpperPIDController.setReference(
                    mUpperDesiredPosition,
                    ControlType.kSmartMotion,
                    ArmConstants.UPPER_SMART_MOTION_SLOT,
                    0,
                    ArbFFUnits.kPercentOut);
        }
    }

    @Override
    public void stop() {
        mLowerLeader.stopMotor();
        mUpperLeader.stopMotor();
    }

    @Override
    public void zero() {

    }

    private void configLowerSparkMax() {
        // AbsoluteEncoder.setZeroOffset(ArmConstants.LOWER_ZERO_OFFSET);
        // AbsoluteEncoder.setInverted(ArmConstants.LOWER_ENCODER_INVERSION);

        mLowerLeader.setIdleMode(IdleMode.kBrake);
        mLowerLeader.setInverted(ArmConstants.LOWER_MOTOR_INVERSION);
        mLowerLeader.setSmartCurrentLimit(ArmConstants.LOWER_CURRENT_LIMIT);
        mLowerLeader.enableVoltageCompensation(Constants.VOLTAGE_COMP);
        // mLowerForwardLimitSwitch.enableLimitSwitch(true);
        // mLowerReverseLimitSwitch.enableLimitSwitch(true);
        // mLowerEncoder.setZeroOffset(ArmConstants.LOWER_ZERO_OFFSET);
        // mLowerEncoder.setInverted(ArmConstants.LOWER_ENCODER_INVERSION);
        mLowerPIDController.setFeedbackDevice(mLowerEncoder);
        mLowerPIDController.setPositionPIDWrappingEnabled(true);
        mLowerPIDController.setPositionPIDWrappingMinInput(ArmConstants.PID_WRAPPING_MIN);
        mLowerPIDController.setPositionPIDWrappingMinInput(ArmConstants.PID_WRAPPING_MAX);
        mLowerPIDController.setFF(ArmConstants.LOWER_KF, ArmConstants.LOWER_SMART_MOTION_SLOT);
        mLowerPIDController.setP(ArmConstants.LOWER_KP, ArmConstants.LOWER_SMART_MOTION_SLOT);
        mLowerPIDController.setI(ArmConstants.LOWER_KI, ArmConstants.LOWER_SMART_MOTION_SLOT);
        mLowerPIDController.setD(ArmConstants.LOWER_KD, ArmConstants.LOWER_SMART_MOTION_SLOT);
        mLowerPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal,
                ArmConstants.LOWER_SMART_MOTION_SLOT);
        mLowerPIDController.setSmartMotionAllowedClosedLoopError(ArmConstants.LOWER_MIN_ERROR,
                ArmConstants.LOWER_SMART_MOTION_SLOT);
        mLowerPIDController.setSmartMotionMinOutputVelocity(ArmConstants.LOWER_MIN_VEL,
                ArmConstants.LOWER_SMART_MOTION_SLOT);
        mLowerPIDController.setSmartMotionMaxVelocity(ArmConstants.LOWER_MAX_VEL, ArmConstants.LOWER_SMART_MOTION_SLOT);
        mLowerPIDController.setSmartMotionMaxAccel(ArmConstants.LOWER_MAX_ACCEL, ArmConstants.LOWER_SMART_MOTION_SLOT);
    }

    private void configUpperSparkMax() {
        mUpperLeader.setIdleMode(IdleMode.kBrake);
        mUpperLeader.setInverted(ArmConstants.UPPER_MOTOR_INVERSION);
        mUpperLeader.setSmartCurrentLimit(ArmConstants.UPPER_CURRENT_LIMIT);
        mUpperLeader.enableVoltageCompensation(Constants.VOLTAGE_COMP);
        // mUpperEncoder.setZeroOffset(ArmConstants.UPPER_ZERO_OFFSET);
        // mUpperEncoder.setInverted(ArmConstants.UPPER_ENCODER_INVERSION);
        mUpperPIDController.setFeedbackDevice(mUpperEncoder);
        mUpperPIDController.setPositionPIDWrappingEnabled(true);
        mUpperPIDController.setPositionPIDWrappingMinInput(ArmConstants.PID_WRAPPING_MIN);
        mUpperPIDController.setPositionPIDWrappingMinInput(ArmConstants.PID_WRAPPING_MAX);
        mUpperPIDController.setFF(ArmConstants.UPPER_KF, ArmConstants.UPPER_SMART_MOTION_SLOT);
        mUpperPIDController.setP(ArmConstants.UPPER_KP, ArmConstants.UPPER_SMART_MOTION_SLOT);
        mUpperPIDController.setI(ArmConstants.UPPER_KI, ArmConstants.UPPER_SMART_MOTION_SLOT);
        mUpperPIDController.setD(ArmConstants.UPPER_KD, ArmConstants.UPPER_SMART_MOTION_SLOT);
        mUpperPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal,
                ArmConstants.UPPER_SMART_MOTION_SLOT);
        mUpperPIDController.setSmartMotionAllowedClosedLoopError(ArmConstants.UPPER_MIN_ERROR,
                ArmConstants.UPPER_SMART_MOTION_SLOT);
        mUpperPIDController.setSmartMotionMinOutputVelocity(ArmConstants.UPPER_MIN_VEL,
                ArmConstants.UPPER_SMART_MOTION_SLOT);
        mUpperPIDController.setSmartMotionMaxVelocity(ArmConstants.UPPER_MAX_VEL, ArmConstants.UPPER_SMART_MOTION_SLOT);
        mUpperPIDController.setSmartMotionMaxAccel(ArmConstants.UPPER_MAX_ACCEL, ArmConstants.UPPER_SMART_MOTION_SLOT);
    }

    public void setArmRampRate(double val) {
        mUpperLeader.setClosedLoopRampRate(val);
        mLowerLeader.setClosedLoopRampRate(val);
    }

    public void moveArm(double lowerOut, double upperOut) {
        mControlState = ControlState.OPEN_LOOP;
        mLowerPercentOut = lowerOut;
        mUpperPercentOut = upperOut;
    }

    public void moveToAngle(double lowerAngle, double upperAngle) {
        mControlState = ControlState.CLOSED_LOOP;
        mLowerDesiredPosition = lowerAngle / ArmConstants.TICKS_TO_DEGREES;
        mUpperDesiredPosition = upperAngle / ArmConstants.TICKS_TO_DEGREES;
    }

    public double[] forKin(Rotation2d[] q) {

        Rotation2d ang_1 = new Rotation2d(state[0]);
        Rotation2d ang_2 = new Rotation2d(state[1]);
        double[] calc;

        // Elbow state
        calc[0] = Rotation2d.fromDegrees(state[0](state[0]))
        state[4] = ArmConstants.LOWER_ARM_LENGTH * Math.sin(Math.toRadians(90 - state[0]));
        state[5] = -1 * ArmConstants.LOWER_ARM_LENGTH * Math.cos(Math.toRadians(90 - state[0]));

        // End-effector state
        state[2] = state[4] + ArmConstants.UPPER_ARM_LENGTH
                * Math.sin(Math.toRadians(90 - state[0]) + Math.toRadians(360 - state[1]));
        state[3] = state[5] + -1 * ArmConstants.UPPER_ARM_LENGTH
                * Math.cos(Math.toRadians(90 - state[0]) + Math.toRadians(360 - state[1]));
    }

    public double[] invKin(double[] pos) {

        
        Pose2d elbow = new Pose2d(ArmConstants.LOWER_ARM_LENGTH * ang_1.fromRadians().getSin(),
                -ArmConstants.LOWER_ARM_LENGTH * ang_1.getCos(), ang_1);


        Pose2d endeff = new Pose2d();
        // Position of target end-effector state
        radius_sq = pos[0] * pos[0] + pos[1] + pos[1];
        radius = Math.sqrt(radius_sq);
        // Angle of target State
        theta = Math.atan2(pos[0], -1 * pos[1]);

        // Use law of cosines to compute elbow angle
        acosarg = (radius_sq - ArmConstants.LOWER_ARM_LENGTH * ArmConstants.LOWER_ARM_LENGTH
                - ArmConstants.UPPER_ARM_LENGTH * ArmConstants.UPPER_ARM_LENGTH)
                / (-2 * ArmConstants.LOWER_ARM_LENGTH * ArmConstants.UPPER_ARM_LENGTH);
        if (acosarg < -1.0)
            elbow_supplement = Math.PI;
        else if (acosarg > 1.0)
            elbow_supplement = 0.0;
        else
            elbow_supplement = Math.acos(acosarg);

        // use law of sines to compute angle at the bottom vertex of the triangle
        // defined by the links
        if (radius > 0.0)
            alpha = Math.asin(ArmConstants.UPPER_ARM_LENGTH * Math.sin(elbow_supplement) / radius);
        else
            alpha = 0.0;

        // compute the two solutions with opposite elbow sign
        s1[0] = Math.toDegrees(theta - alpha);
        s1[1] = Math.toDegrees(Math.PI - elbow_supplement);

        s2[0] = Math.toDegrees(theta + alpha);
        s2[1] = Math.toDegrees(elbow_supplement - Math.PI);

        if (Math.abs(s1[0]) > 160 || Math.abs(s1[1]) > 160) {
            sf[0] = s2[0];
            sf[1] = s2[1];
        } else {
            sf[0] = s1[0];
            sf[1] = s1[1];
        }
        return sf;
    }
}
