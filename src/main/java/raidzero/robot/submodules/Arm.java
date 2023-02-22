package raidzero.robot.submodules;

import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import raidzero.robot.Constants;
import raidzero.robot.Constants.ArmConstants;
import raidzero.robot.wrappers.LazyCANSparkMax;

public class Arm extends Submodule {

    // Motor Control Constants
    private DoubleJointedArm Controller = new DoubleJointedArm();
    private ControlState mControlState = ControlState.OPEN_LOOP;
    private double outputOpenLoop = 0.0;
    private double mLowerPercentOut = 0.0;
    private double mUpperPercentOut = 0.0;
    private double mLowerDesiredPosition = 0.0;
    private double mUpperDesiredPosition = 0.0;

    // Multi-Staged Movement Constants
    private int stage = 0;
    private double mLowerWaypointPositions[] = { 0.0, 0.0, 0.0 };
    private double mUpperWaypointPositions[] = { 0.0, 0.0, 0.0 };
    // Intermediate State Constants
    private double[] xWaypointPositions = { 0, 0, 0 };
    private double[] yWaypointPositions = { 0, 0, 0 };
    private double[] wristWaypointPositions = { 0, 0, 0 };

    // Absolute Encoder Adjustment Constants
    public double drift = 0.0; // degrees
    public double driftTolerance = 10.0;
    public double dResets = 0.0;

    // State of Proximal and Distal Links
    private Pose2d[] state;
    private Rotation2d[] q = {
            Rotation2d.fromDegrees(90),
            Rotation2d.fromDegrees(0) };

    // Motor Declaration
    private final LazyCANSparkMax mLowerLeader = new LazyCANSparkMax(ArmConstants.LOWER_LEADER_ID,
            MotorType.kBrushless);
    private final LazyCANSparkMax mLowerFollower = new LazyCANSparkMax(ArmConstants.LOWER_FOLLOWER_ID,
            MotorType.kBrushless);
    private final LazyCANSparkMax mUpperLeader = new LazyCANSparkMax(ArmConstants.UPPER_LEADER_ID,
            MotorType.kBrushless);
    private final LazyCANSparkMax mUpperFollower = new LazyCANSparkMax(ArmConstants.UPPER_FOLLOWER_ID,
            MotorType.kBrushless);
    private Wrist wrist = Wrist.getInstance();
    private final SparkMaxPIDController mLowerPIDController = mLowerLeader.getPIDController();
    private final SparkMaxPIDController mUpperPIDController = mUpperLeader.getPIDController();

    // Limit Switches
    private final SparkMaxLimitSwitch mLowerForwardLimitSwitch = mLowerLeader
            .getForwardLimitSwitch(ArmConstants.LOWER_FORWARD_LIMIT_TYPE);
    private final SparkMaxLimitSwitch mLowerReverseLimitSwitch = mLowerLeader
            .getReverseLimitSwitch(ArmConstants.LOWER_REVERSE_LIMIT_TYPE);
    private final SparkMaxLimitSwitch mUpperForwardLimitSwitch = mUpperLeader
            .getForwardLimitSwitch(ArmConstants.LOWER_FORWARD_LIMIT_TYPE);
    private final SparkMaxLimitSwitch mUpperReverseLimitSwitch = mUpperLeader
            .getReverseLimitSwitch(ArmConstants.LOWER_REVERSE_LIMIT_TYPE);

    // Motor Encoders
    private final SparkMaxAbsoluteEncoder mLowerAbsoluteEncoder = mLowerLeader
            .getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    private final SparkMaxAbsoluteEncoder mUpperAbsoluteEncoder = mUpperLeader
            .getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    private final RelativeEncoder mLowerEncoder = mLowerLeader.getEncoder();
    private final RelativeEncoder mUpperEncoder = mUpperLeader.getEncoder();

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

    @Override
    public void onInit() {
        mLowerLeader.restoreFactoryDefaults();
        mLowerFollower.restoreFactoryDefaults();
        mUpperLeader.restoreFactoryDefaults();
        mUpperFollower.restoreFactoryDefaults();

        mLowerFollower.follow(mLowerLeader, false);
        mUpperFollower.follow(mUpperLeader, false);

        configLowerSparkMax();
        configUpperSparkMax();

        zero();
        stage = 0;
        wrist.onInit();
    }

    @Override
    public void onStart(double timestamp) {
        // Proximal Angle Update
        q[0] = Rotation2d.fromDegrees(90).minus(Rotation2d
                .fromDegrees(mLowerEncoder.getPosition() * ArmConstants.TICKS_TO_DEGREES));
        // Distal Angle Update
        q[1] = Rotation2d
                .fromDegrees(90 + q[0].getDegrees() - mUpperEncoder.getPosition() * ArmConstants.TICKS_TO_DEGREES)
                .unaryMinus();

        state[0] = new Pose2d(forKin(q)[0], forKin(q)[1], q[0]); // Proximal
        state[1] = new Pose2d(forKin(q)[2], forKin(q)[3], q[1]); // Distal
    }

    @Override
    public void update(double timestamp) {
        // Proximal Angle Update
        q[0] = Rotation2d.fromDegrees(90).minus(Rotation2d
                .fromDegrees(mLowerEncoder.getPosition() * ArmConstants.TICKS_TO_DEGREES));
        // Distal Angle Update
        q[1] = Rotation2d
                .fromDegrees(90 + q[0].getDegrees() - mUpperEncoder.getPosition() * ArmConstants.TICKS_TO_DEGREES)
                .unaryMinus();

        // Absolute Encoder Sanity Checks
        // Rotation2d[] q = {
        // Rotation2d.fromDegrees(90).minus(Rotation2d
        // .fromDegrees(mLowerEncoder.getPosition() * ArmConstants.TICKS_TO_DEGREES)),
        // lowerSanityCheck(
        // Rotation2d.fromRadians(mLowerAbsoluteEncoder.getPosition()),
        // Rotation2d.fromDegrees(90).minus(Rotation2d
        // .fromDegrees(mLowerEncoder.getPosition() * ArmConstants.TICKS_TO_DEGREES))),
        // upperSanityCheck(
        // Rotation2d.fromDegrees(mUpperEncoder.getPosition()
        // *ArmConstants.TICKS_TO_DEGREES)
        // .unaryMinus(),
        // Rotation2d.fromDegrees(mUpperEncoder.getPosition() *
        // ArmConstants.TICKS_TO_DEGREES)
        // .unaryMinus()) };

        state[0] = new Pose2d(forKin(q)[0], forKin(q)[1], q[0]); // Proximal
        state[1] = new Pose2d(forKin(q)[2], forKin(q)[3], q[1]); // Distal

        // Link Angles
        SmartDashboard.putNumber("Proximal Absolute Angle", Math.toDegrees(mLowerAbsoluteEncoder.getPosition()) + 90);
        SmartDashboard.putNumber("Proximal Angle", state[0].getRotation().getDegrees());
        SmartDashboard.putNumber("Distal Angle", state[1].getRotation().getDegrees());

        // Link Positions
        SmartDashboard.putNumber("Proximal X ", state[0].getX());
        SmartDashboard.putNumber("Proximal Y ", state[0].getY());
        SmartDashboard.putNumber("Distal X", state[1].getX());
        SmartDashboard.putNumber("Distal Y", state[1].getY());
        SmartDashboard.putNumber("Drift",
                Math.toDegrees(mLowerAbsoluteEncoder.getPosition()) + 90 - state[0].getRotation().getDegrees());
        SmartDashboard.putNumber("Resets", dResets);

        SmartDashboard.putNumber("Wrist Rotations", wrist.getRotations());
        SmartDashboard.putNumber("Wrist Degrees", wrist.getAngle().getDegrees());
        SmartDashboard.putNumber("TooFast", tooFasttooFurious());

        // Two Pronged Movement
        if (stage > 0) {
            // Attempt approximate linear motion
            // Check for Intermediate Error and Proceed to Staged Target
            if (Math.abs(state[1].getX() - xWaypointPositions[stage - 1]) < 0.1
                    && Math.abs(state[1].getY() - yWaypointPositions[stage - 1]) < 0.1) {
                // Stage Check: Within Range, Proceed to Following Stage
                if (stage < mLowerWaypointPositions.length) {
                    moveToPoint(xWaypointPositions[stage], yWaypointPositions[stage], wristWaypointPositions[stage]);
                    System.out.println("Moving to point");
                    stage++;
                    stage %= xWaypointPositions.length;
                }
            }
        }
    }

    @Override
    public void run() {
        if (mControlState == ControlState.OPEN_LOOP) {
            if (stage > 0) {
                mLowerLeader.stopMotor();
                mUpperLeader.stopMotor();
                stage = 0;
            }
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
        wrist.run();
    }

    @Override
    public void stop() {
        mLowerLeader.stopMotor();
        mUpperLeader.stopMotor();
        stage = 0;
        wrist.stop();
    }

    @Override
    public void zero() {
        // mLowerEncoder.setPosition(0);
        // mUpperEncoder.setPosition(0);
        // wrist.zero();
    }

    private void configLowerSparkMax() {
        mLowerLeader.setIdleMode(IdleMode.kBrake);
        mLowerLeader.setInverted(ArmConstants.LOWER_MOTOR_INVERSION);
        mLowerLeader.setSmartCurrentLimit(ArmConstants.LOWER_CURRENT_LIMIT);
        mLowerLeader.enableVoltageCompensation(Constants.VOLTAGE_COMP);
        mLowerForwardLimitSwitch.enableLimitSwitch(true);
        mLowerReverseLimitSwitch.enableLimitSwitch(true);

        mLowerAbsoluteEncoder.setInverted(ArmConstants.LOWER_ABSOLUTE_ENCODER_INVERSION);
        mLowerAbsoluteEncoder.setPositionConversionFactor(ArmConstants.LOWER_ABS_POSITION_CONVERSION_FACTOR);
        mLowerAbsoluteEncoder.setZeroOffset(ArmConstants.LOWER_ZERO_OFFSET);

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

        // Periodic Frame Period
        mLowerLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
        mLowerLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
        mLowerLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);
    }

    private void configUpperSparkMax() {
        mUpperLeader.setIdleMode(IdleMode.kBrake);
        mUpperLeader.setInverted(ArmConstants.UPPER_MOTOR_INVERSION);
        mUpperLeader.setSmartCurrentLimit(ArmConstants.UPPER_CURRENT_LIMIT);
        mUpperLeader.enableVoltageCompensation(Constants.VOLTAGE_COMP);
        mUpperForwardLimitSwitch.enableLimitSwitch(ArmConstants.UPPER_LIMIT_ENABLED);
        mUpperReverseLimitSwitch.enableLimitSwitch(ArmConstants.UPPER_LIMIT_ENABLED);

        mUpperAbsoluteEncoder.setInverted(ArmConstants.UPPER_ABSOLUTE_ENCODER_INVERSION);
        mUpperAbsoluteEncoder.setPositionConversionFactor(ArmConstants.UPPER_ABS_POSITION_CONVERSION_FACTOR);
        mUpperAbsoluteEncoder.setZeroOffset(ArmConstants.UPPER_ZERO_OFFSET);

        mUpperLeader.enableSoftLimit(LazyCANSparkMax.SoftLimitDirection.kForward, true);
        mUpperLeader.enableSoftLimit(LazyCANSparkMax.SoftLimitDirection.kReverse, true);
        mUpperLeader.setSoftLimit(LazyCANSparkMax.SoftLimitDirection.kReverse,
                (float) ArmConstants.UPPER_REV_SOFTLIMIT);
        mUpperLeader.setSoftLimit(LazyCANSparkMax.SoftLimitDirection.kForward,
                (float) ArmConstants.UPPER_FWD_SOFTLIMIT);

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

        // Periodic Frame Period
        mUpperLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
        mUpperLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
        mUpperLeader.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);
    }

    public void setArmRampRate(double val) {
        mUpperLeader.setClosedLoopRampRate(val);
        mLowerLeader.setClosedLoopRampRate(val);
    }

    public Pose2d[] getState() {
        return state;
    }

    public Wrist getWrist() {
        return wrist;
    }

    public double getStage() {
        return stage;
    }

    /**
     * Converts angle such that result is bound between -2pi and 0
     */
    public double angleConv(double org) {
        if (Math.signum(org) > 0)
            return -360 + org;
        else
            return org;
    }

    public double tooFasttooFurious() {
        if (Math.abs(state[1].getX()) > 0.3)
            return 0.1;
        else
            return 1;
    }

    /**
     * Open loop Arm Control
     */
    public void moveArm(double lowerOut, double upperOut) {
        mControlState = ControlState.OPEN_LOOP;
        mLowerPercentOut = lowerOut;
        mUpperPercentOut = upperOut;
    }

    /**
     * Closed loop Arm + Wrist Control (Target Angle Array, Target Wrist Angle)
     */
    public void moveToAngle(double targetAngles[], double wristAngle) {
        mControlState = ControlState.CLOSED_LOOP;
        mLowerDesiredPosition = (90 - targetAngles[0]) / ArmConstants.TICKS_TO_DEGREES;
        mUpperDesiredPosition = (90 + targetAngles[0] + targetAngles[1]) / ArmConstants.TICKS_TO_DEGREES;

        wrist.setDesiredAngle(calculateWristRelativeAngle(wristAngle));
    }

    public void configSmartMotionConstraints(double lowerMaxVel, double lowerMaxAccel, double upperMaxVel,
            double upperMaxAccel) {
        mLowerPIDController.setSmartMotionMaxVelocity(lowerMaxVel, ArmConstants.LOWER_SMART_MOTION_SLOT);
        mLowerPIDController.setSmartMotionMaxAccel(lowerMaxAccel, ArmConstants.LOWER_SMART_MOTION_SLOT);
        mUpperPIDController.setSmartMotionMaxVelocity(upperMaxVel, ArmConstants.UPPER_SMART_MOTION_SLOT);
        mUpperPIDController.setSmartMotionMaxAccel(upperMaxAccel, ArmConstants.UPPER_SMART_MOTION_SLOT);
    }

    /**
     * Closed loop Arm + Wrist Control (Target Lower Angle, Target Upper Angle,
     * Associated Parallel Wrist Angle)
     */
    public void moveToAngle(double lowerAngle, double upperAngle) {
        mControlState = ControlState.CLOSED_LOOP;
        double targetWristAngle = calculateWristAbsoluteAngle(wrist.getAngle().getDegrees());
        double[] targetAngles = { lowerAngle, upperAngle };
        moveToAngle(targetAngles, targetWristAngle);
    }

    public double calculateWristAbsoluteAngle(double relativeAngle) {
        return relativeAngle - (mUpperEncoder.getPosition() * ArmConstants.TICKS_TO_DEGREES);
    }

    public double calculateWristRelativeAngle(double targetAngle) {
        double relativeAngle = targetAngle - (mUpperDesiredPosition * ArmConstants.TICKS_TO_DEGREES);
        System.out.println(relativeAngle);
        return relativeAngle;
    }

    private void attemptLinearMotion() {
        double rightTriangleDifference = ArmConstants.LOWER_ARM_LENGTH * ArmConstants.LOWER_ARM_LENGTH
                - ArmConstants.UPPER_ARM_LENGTH * ArmConstants.UPPER_ARM_LENGTH
                - state[1].getX() * state[1].getX() - state[1].getY() * state[1].getY();
        double R = 1 / (2 * state[1].getX() * state[1].getX() + state[1].getY() * state[1].getY())
                * rightTriangleDifference;
        double ratio = Math.max(Math.abs(R / (1 + R)), 3);
        double lower_arm_target_velocity = ArmConstants.TOTAL_MAX_VEL;
    }

    public void moveToPoint(double target_x, double target_y, double wristAngle) {
        mControlState = ControlState.CLOSED_LOOP;
        moveToAngle(invKin(target_x, target_y), wristAngle);
    }

    // TODO: Add Kalman Filter to sanity check here:
    public Rotation2d lowerSanityCheck(Rotation2d abs, Rotation2d rel) {
        if (Math.abs(rel.minus(abs).getDegrees()) > driftTolerance && abs.getRadians() <= Math.PI
                && abs.getRadians() >= 0) {
            mLowerEncoder
                    .setPosition(Rotation2d.fromDegrees(90).minus(abs).getDegrees() / ArmConstants.TICKS_TO_DEGREES);
            dResets++;
            return abs;
        }
        return rel;
    }

    // TODO: Add Kalman Filter to sanity check here:
    public Rotation2d upperSanityCheck(Rotation2d abs, Rotation2d rel) {
        if (Math.abs(abs.minus(rel).getDegrees()) > driftTolerance) {
            mUpperEncoder.setPosition(abs.unaryMinus().getDegrees() / ArmConstants.TICKS_TO_DEGREES);
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
        pos[2] = pos[0] + ArmConstants.UPPER_ARM_LENGTH
                * q[0].plus(q[1]).getCos();
        pos[3] = pos[1] + ArmConstants.UPPER_ARM_LENGTH
                * q[0].plus(q[1]).getSin();

        return pos;
    }

    public double[] invKin(double target_x, double target_y) {
        // Prevent upper arm from crossing the y-axis
        return invKin(target_x, target_y, target_x < 0);
    }

    public double[] invKin(double target_x, double target_y, boolean bq1) {
        // Position of target end-effector state
        double radius_sq = target_x * target_x + target_y * target_y;
        double radius = Math.sqrt(radius_sq);

        // Angle of target State
        double theta = Math.atan2(target_y, target_x);

        // Use law of cosines to compute elbow angle
        double elbow_supplement = 0.0;
        double acosarg = (radius_sq - ArmConstants.LOWER_ARM_LENGTH * ArmConstants.LOWER_ARM_LENGTH
                - ArmConstants.UPPER_ARM_LENGTH * ArmConstants.UPPER_ARM_LENGTH)
                / (-2 * ArmConstants.LOWER_ARM_LENGTH * ArmConstants.UPPER_ARM_LENGTH);
        if (acosarg < -1.0)
            elbow_supplement = Math.PI;
        else if (acosarg > 1.0)
            elbow_supplement = 0.0;
        else
            elbow_supplement = Math.acos(acosarg);

        // Use law of sines to compute angle at the bottom vertex of the triangle
        // defined by the links
        double alpha = 0;
        if (radius > 0.0)
            alpha = Math.asin(ArmConstants.UPPER_ARM_LENGTH * Math.sin(elbow_supplement) / radius);
        else
            alpha = 0.0;

        // Compute the two solutions with opposite elbow sign
        double[] s1 = { Math.toDegrees(theta - alpha), angleConv(Math.toDegrees(Math.PI - elbow_supplement)) };
        double[] s2 = { Math.toDegrees(theta + alpha), angleConv(Math.toDegrees(elbow_supplement - Math.PI)) };

        // // Elbow Checks
        // if (Math.abs(s1[0] - state[0].getRotation().getDegrees()) < 5) {
        // return s1;
        // } else {
        // if (s1[1] > -180 && state[1].getRotation().getDegrees() > -180) {
        // return s1;
        // } else if (s1[1] < -180 && state[1].getRotation().getDegrees() < -180) {
        // return s1;
        // } else {
        // return s2;
        // }
        // }
        // if (bq1)
        // return s1;
        // else
        // return s2;

        // Check for wacko solutions
        if (Math.signum(s1[0]) < 0) {
            return s2;
        } else
            return s1;

        // //Alternative
        // double abs_elbow_angle =
        // MathTools.lawOfCosines(ArmConstants.LOWER_ARM_LENGTH,
        // ArmConstants.UPPER_ARM_LENGTH, radius);
        // double lower_interior_angle =
        // MathTools.lawOfCosines(ArmConstants.LOWER_ARM_LENGTH,radius,
        // ArmConstants.UPPER_ARM_LENGTH);

        // //Calculate the two possible lower arm angles, eliminate them for the
        // alternative
        // double lower_negative = theta + lower_interior_angle > Math.PI -
        // Math.toRadians(ArmConstants.LOWER_MAX_ANGLE) ? theta - lower_interior_angle:
        // theta + lower_interior_angle;
        // double lower_positive = theta - lower_interior_angle <
        // Math.toRadians(ArmConstants.LOWER_MAX_ANGLE) ? theta + lower_interior_angle:
        // theta - lower_interior_angle;

        // //Return the solution given where the elbow is: Note, if there are not two
        // solutions, the elbow
        // //will go to the only possible location
        // double lower_solution = positiveElbow ? lower_positive : lower_negative;
        // double upper_solution = lower_solution > Math.PI/2 ? +abs_elbow_angle-Math.PI
        // : -Math.PI-abs_elbow_angle;
        // double[] solution = new double[] {Math.toDegrees(lower_solution),
        // angleConv(Math.toDegrees(upper_solution))} ;
        // //Compare the motion of the upper arm two solutions- we want to avoid having
        // it cross the midline if possible

        // return solution;
    }

    private double calcSpeedRatio() {
        double radius_sq = state[1].getX() * state[1].getX() + state[1].getY() * state[1].getY();
        double square_diff = ArmConstants.LOWER_ARM_LENGTH * ArmConstants.LOWER_ARM_LENGTH
                - ArmConstants.UPPER_ARM_LENGTH * ArmConstants.UPPER_ARM_LENGTH;
        return ArmConstants.UPPER_ARM_LENGTH / (2 * ArmConstants.LOWER_ARM_LENGTH) * (radius_sq)
                / (radius_sq + square_diff);
    }

    public void moveTwoPronged(double inter_x, double inter_y, double inter_wrist,
            double target_x, double target_y, double target_wrist) {
        stage = 1;
        xWaypointPositions = new double[2];
        yWaypointPositions = new double[2];
        wristWaypointPositions = new double[2];
        xWaypointPositions[0] = inter_x;
        xWaypointPositions[1] = target_x;
        yWaypointPositions[0] = inter_y;
        yWaypointPositions[1] = target_y;
        wristWaypointPositions[0] = inter_wrist;
        wristWaypointPositions[1] = target_wrist;
        moveToPoint(inter_x, inter_y, inter_wrist);
    }

    public void moveThreePronged(double inter_x, double inter_y, double inter_wrist,
            double inter_x2, double inter_y2, double inter_wrist2,
            double target_x, double target_y, double target_wrist) {
        stage = 1;
        xWaypointPositions = new double[3];
        yWaypointPositions = new double[3];
        wristWaypointPositions = new double[3];
        xWaypointPositions[0] = inter_x;
        xWaypointPositions[1] = inter_x2;
        xWaypointPositions[2] = target_x;

        yWaypointPositions[0] = inter_y;
        yWaypointPositions[1] = inter_y2;
        yWaypointPositions[2] = target_y;

        wristWaypointPositions[0] = inter_wrist;
        wristWaypointPositions[1] = inter_wrist2;
        wristWaypointPositions[2] = target_wrist;
        moveToPoint(inter_x, inter_y, inter_wrist);
    }

    public void goHome() {
        configSmartMotionConstraints(
                ArmConstants.LOWER_MAX_VEL * 2.0,
                ArmConstants.LOWER_MAX_ACCEL * 2.0,
                ArmConstants.UPPER_MAX_VEL * 1.25,
                ArmConstants.UPPER_MAX_ACCEL * 1.25);

        if (state[1].getY() < 0.15) {
            moveTwoPronged(state[1].getX(), 0.25, 0, 0, 0.15, 0);
        } else if (state[1].getY() > 0.5 && Math.abs(state[1].getX()) > 0.3) {
            System.out.println("Safety one " + 0.05 * Math.signum(state[1].getX()));
            moveThreePronged(0.05 * Math.signum(state[1].getX()), state[1].getY() + .1, 0,
                    0.15 * Math.signum(state[1].getX()), 0.5, -70, 0.0, 0.15, 0);
            // } else if (state[1].getY() < 0.3 && Math.abs(state[1].getX()) > 0.3) {
            // moveTwoPronged(0.3 * Math.signum(state[1].getX()), state[1].getY() + .1, 0,
            // 0.0, 0.15, 0);
            // System.out.println("Safety two");
        } else {
            moveToAngle(new double[] { 90, -180 }, 0);
        }
    }
}
