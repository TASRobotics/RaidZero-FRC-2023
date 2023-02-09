package raidzero.robot.submodules;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import raidzero.robot.Constants;
import raidzero.robot.Constants.ArmConstants;
import raidzero.robot.submodules.DoubleJointedArm;
import raidzero.robot.wrappers.LazyCANSparkMax;

public class Arm extends Submodule {

  private DoubleJointedArm Controller = new DoubleJointedArm();

  private ControlState mControlState = ControlState.OPEN_LOOP;
  private double outputOpenLoop = 0.0;

  private double mLowerPercentOut = 0.0;
  private double mUpperPercentOut = 0.0;
  private double mLowerDesiredPosition = 0.0;
  private double mUpperDesiredPosition = 0.0;

  public double drift = 0.0; // degrees
  public double driftTolerance = 5.0;
  public double dResets = 0.0;

  // State of Proximal and Distal Links
  private Pose2d[] state;

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
    OPEN_LOOP,
    CLOSED_LOOP,
  }

  private final LazyCANSparkMax mLowerLeader = new LazyCANSparkMax(
    ArmConstants.LOWER_LEADER_ID,
    MotorType.kBrushless
  );
  // private final LazyCANSparkMax mLowerFollower = new
  // LazyCANSparkMax(ArmConstants.LOWER_FOLLOWER_ID,
  // MotorType.kBrushless);
  private final LazyCANSparkMax mUpperLeader = new LazyCANSparkMax(
    ArmConstants.UPPER_LEADER_ID,
    MotorType.kBrushless
  );
  // private final LazyCANSparkMax mUpperFollower = new
  // LazyCANSparkMax(ArmConstants.UPPER_FOLLOWER_ID,
  // MotorType.kBrushless);

  // private final SparkMaxLimitSwitch mLowerForwardLimitSwitch = mLowerLeader
  // .getForwardLimitSwitch(ArmConstants.LOWER_FORWARD_LIMIT_TYPE);
  // private final SparkMaxLimitSwitch mLowerReverseLimitSwitch = mLowerLeader
  // .getReverseLimitSwitch(ArmConstants.LOWER_REVERSE_LIMIT_TYPE);

  private final SparkMaxAbsoluteEncoder mLowerAbsoluteEncoder = mLowerLeader.getAbsoluteEncoder(
    SparkMaxAbsoluteEncoder.Type.kDutyCycle
  );
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

    mLowerLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    mLowerLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    mLowerLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);

    mLowerLeader.burnFlash();
    mUpperLeader.burnFlash();
  }

  @Override
  public void onStart(double timestamp) {}

  @Override
  public void update(double timestamp) {
    Rotation2d[] q = {
      // Rotation2d.fromDegrees(90).minus(Rotation2d
      // .fromDegrees(mLowerEncoder.getPosition() * ArmConstants.TICKS_TO_DEGREES)),
      lowerSanityCheck(
        Rotation2d.fromRadians(mLowerAbsoluteEncoder.getPosition()),
        Rotation2d
          .fromDegrees(90)
          .minus(
            Rotation2d.fromDegrees(
              mLowerEncoder.getPosition() * ArmConstants.TICKS_TO_DEGREES
            )
          )
      ),
      upperSanityCheck(
        Rotation2d
          .fromDegrees(
            mUpperEncoder.getPosition() * ArmConstants.TICKS_TO_DEGREES
          )
          .unaryMinus(),
        Rotation2d
          .fromDegrees(
            mUpperEncoder.getPosition() * ArmConstants.TICKS_TO_DEGREES
          )
          .unaryMinus()
      ),
    };
    state[0] = new Pose2d(forKin(q)[0], forKin(q)[1], q[0]); // Proximal
    state[1] = new Pose2d(forKin(q)[2], forKin(q)[3], q[1]); // Distal

    SmartDashboard.putNumber(
      "Absolute Angle",
      Math.toDegrees(mLowerAbsoluteEncoder.getPosition())
    );
    SmartDashboard.putNumber(
      "Proximal Angle",
      state[0].getRotation().getDegrees()
    );
    SmartDashboard.putNumber(
      "Distal Angle",
      state[1].getRotation().getDegrees()
    );
    SmartDashboard.putNumber("Proximal X ", state[0].getX());
    SmartDashboard.putNumber("Proximal Y ", state[0].getY());
    SmartDashboard.putNumber("Distal X", state[1].getX());
    SmartDashboard.putNumber("Distal Y", state[1].getY());
    SmartDashboard.putNumber(
      "Drift",
      Math.toDegrees(mLowerAbsoluteEncoder.getPosition()) -
      state[0].getRotation().getDegrees()
    );
    SmartDashboard.putNumber("Resets", dResets);
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
        ArbFFUnits.kPercentOut
      );
      mUpperPIDController.setReference(
        mUpperDesiredPosition,
        ControlType.kSmartMotion,
        ArmConstants.UPPER_SMART_MOTION_SLOT,
        0,
        ArbFFUnits.kPercentOut
      );
    }
  }

  @Override
  public void stop() {
    mLowerLeader.stopMotor();
    mUpperLeader.stopMotor();
  }

  @Override
  public void zero() {}

  private void configLowerSparkMax() {
    mLowerLeader.setIdleMode(IdleMode.kBrake);
    mLowerLeader.setInverted(ArmConstants.LOWER_MOTOR_INVERSION);
    mLowerLeader.setSmartCurrentLimit(ArmConstants.LOWER_CURRENT_LIMIT);
    mLowerLeader.enableVoltageCompensation(Constants.VOLTAGE_COMP);
    // mLowerForwardLimitSwitch.enableLimitSwitch(true);
    // mLowerReverseLimitSwitch.enableLimitSwitch(true);

    mLowerAbsoluteEncoder.setInverted(ArmConstants.ABSOLUTE_ENCODER_INVERSION);
    mLowerAbsoluteEncoder.setPositionConversionFactor(
      ArmConstants.LOWER_ABS_POSITION_CONVERSION_FACTOR
    );
    mLowerAbsoluteEncoder.setZeroOffset(ArmConstants.LOWER_ZERO_OFFSET);

    mLowerPIDController.setFeedbackDevice(mLowerEncoder);
    mLowerPIDController.setPositionPIDWrappingEnabled(true);
    mLowerPIDController.setPositionPIDWrappingMinInput(
      ArmConstants.PID_WRAPPING_MIN
    );
    mLowerPIDController.setPositionPIDWrappingMinInput(
      ArmConstants.PID_WRAPPING_MAX
    );
    mLowerPIDController.setFF(
      ArmConstants.LOWER_KF,
      ArmConstants.LOWER_SMART_MOTION_SLOT
    );
    mLowerPIDController.setP(
      ArmConstants.LOWER_KP,
      ArmConstants.LOWER_SMART_MOTION_SLOT
    );
    mLowerPIDController.setI(
      ArmConstants.LOWER_KI,
      ArmConstants.LOWER_SMART_MOTION_SLOT
    );
    mLowerPIDController.setD(
      ArmConstants.LOWER_KD,
      ArmConstants.LOWER_SMART_MOTION_SLOT
    );
    mLowerPIDController.setSmartMotionAccelStrategy(
      AccelStrategy.kTrapezoidal,
      ArmConstants.LOWER_SMART_MOTION_SLOT
    );
    mLowerPIDController.setSmartMotionAllowedClosedLoopError(
      ArmConstants.LOWER_MIN_ERROR,
      ArmConstants.LOWER_SMART_MOTION_SLOT
    );
    mLowerPIDController.setSmartMotionMinOutputVelocity(
      ArmConstants.LOWER_MIN_VEL,
      ArmConstants.LOWER_SMART_MOTION_SLOT
    );
    mLowerPIDController.setSmartMotionMaxVelocity(
      ArmConstants.LOWER_MAX_VEL,
      ArmConstants.LOWER_SMART_MOTION_SLOT
    );
    mLowerPIDController.setSmartMotionMaxAccel(
      ArmConstants.LOWER_MAX_ACCEL,
      ArmConstants.LOWER_SMART_MOTION_SLOT
    );
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
    mUpperPIDController.setPositionPIDWrappingMinInput(
      ArmConstants.PID_WRAPPING_MIN
    );
    mUpperPIDController.setPositionPIDWrappingMinInput(
      ArmConstants.PID_WRAPPING_MAX
    );
    mUpperPIDController.setFF(
      ArmConstants.UPPER_KF,
      ArmConstants.UPPER_SMART_MOTION_SLOT
    );
    mUpperPIDController.setP(
      ArmConstants.UPPER_KP,
      ArmConstants.UPPER_SMART_MOTION_SLOT
    );
    mUpperPIDController.setI(
      ArmConstants.UPPER_KI,
      ArmConstants.UPPER_SMART_MOTION_SLOT
    );
    mUpperPIDController.setD(
      ArmConstants.UPPER_KD,
      ArmConstants.UPPER_SMART_MOTION_SLOT
    );
    mUpperPIDController.setSmartMotionAccelStrategy(
      AccelStrategy.kTrapezoidal,
      ArmConstants.UPPER_SMART_MOTION_SLOT
    );
    mUpperPIDController.setSmartMotionAllowedClosedLoopError(
      ArmConstants.UPPER_MIN_ERROR,
      ArmConstants.UPPER_SMART_MOTION_SLOT
    );
    mUpperPIDController.setSmartMotionMinOutputVelocity(
      ArmConstants.UPPER_MIN_VEL,
      ArmConstants.UPPER_SMART_MOTION_SLOT
    );
    mUpperPIDController.setSmartMotionMaxVelocity(
      ArmConstants.UPPER_MAX_VEL,
      ArmConstants.UPPER_SMART_MOTION_SLOT
    );
    mUpperPIDController.setSmartMotionMaxAccel(
      ArmConstants.UPPER_MAX_ACCEL,
      ArmConstants.UPPER_SMART_MOTION_SLOT
    );
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
    mLowerDesiredPosition = (90 - lowerAngle) / ArmConstants.TICKS_TO_DEGREES;
    mUpperDesiredPosition = -upperAngle / ArmConstants.TICKS_TO_DEGREES;
  }

  public Pose2d[] getState() {
    return state;
  }

  // TODO: Add Kalman Filter to sanity check here:
  public Rotation2d lowerSanityCheck(Rotation2d abs, Rotation2d rel) {
    if (
      Math.abs(rel.minus(abs).getDegrees()) > driftTolerance &&
      abs.getRadians() <= Math.PI &&
      abs.getRadians() >= 0
    ) {
      mLowerEncoder.setPosition(
        Rotation2d.fromDegrees(90).minus(abs).getDegrees() /
        ArmConstants.TICKS_TO_DEGREES
      );
      dResets++;
      return abs;
    }
    return rel;
  }

  // TODO: Add Kalman Filter to sanity check here:
  public Rotation2d upperSanityCheck(Rotation2d abs, Rotation2d rel) {
    if (Math.abs(abs.minus(rel).getDegrees()) > driftTolerance) {
      mUpperEncoder.setPosition(
        abs.unaryMinus().getDegrees() / ArmConstants.TICKS_TO_DEGREES
      );
      return abs;
    }
    return rel;
  }

  public double[] forKin(Rotation2d[] q) {
    double[] pos = new double[state.length * 2];

    // Proximal Position
    pos[0] = ArmConstants.LOWER_ARM_LENGTH * q[0].getCos();
    pos[1] = ArmConstants.LOWER_ARM_LENGTH * q[0].getSin();

    // Distal Position
    pos[2] = pos[0] + ArmConstants.UPPER_ARM_LENGTH * q[0].plus(q[1]).getCos();
    pos[3] = pos[1] + ArmConstants.UPPER_ARM_LENGTH * q[0].plus(q[1]).getSin();

    return pos;
  }

  public double[] invKin(double[] target) {
    // Position of target end-effector state
    double radius_sq = target[0] * target[0] + target[1] * target[1];
    double radius = Math.sqrt(radius_sq);

    // Angle of target State
    double theta = Math.atan2(target[1], target[0]);

    // Use law of cosines to compute elbow angle
    double elbow_supplement = 0.0;
    double acosarg =
      (
        radius_sq -
        ArmConstants.LOWER_ARM_LENGTH *
        ArmConstants.LOWER_ARM_LENGTH -
        ArmConstants.UPPER_ARM_LENGTH *
        ArmConstants.UPPER_ARM_LENGTH
      ) /
      (-2 * ArmConstants.LOWER_ARM_LENGTH * ArmConstants.UPPER_ARM_LENGTH);
    if (acosarg < -1.0) elbow_supplement = Math.PI; else if (
      acosarg > 1.0
    ) elbow_supplement = 0.0; else elbow_supplement = Math.acos(acosarg);

    // Use law of sines to compute angle at the bottom vertex of the triangle
    // defined by the links
    double alpha = 0;
    if (radius > 0.0) alpha =
      Math.asin(
        ArmConstants.UPPER_ARM_LENGTH * Math.sin(elbow_supplement) / radius
      ); else alpha = 0.0;

    // Compute the two solutions with opposite elbow sign
    double[] s1 = {
      Math.toDegrees(theta - alpha),
      Math.toDegrees(Math.PI - elbow_supplement),
    };
    double[] s2 = {
      Math.toDegrees(theta + alpha),
      Math.toDegrees(elbow_supplement - Math.PI),
    };

    // Check for wacko solutions
    if (
      (Math.signum(s1[0]) > 90 && Math.signum(s1[1]) > 0) ||
      (Math.signum(s1[0]) < 90 && Math.signum(s1[1]) < 0)
    ) {
      return s2;
    } else return s1;
    // Compare elbow angle solutions, find closest angle to move to
    // if (Math.abs(s1[0] - state[0].getRotation().getDegrees()) < Math
    // .abs(s2[0] - state[0].getRotation().getDegrees())) {
    // return s1;
    // } else
    // return s2;

  }
}
