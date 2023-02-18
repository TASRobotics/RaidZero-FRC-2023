package raidzero.robot.submodules;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import raidzero.robot.Constants;
import raidzero.robot.Constants.IntakeConstants;
import raidzero.robot.wrappers.LazyCANSparkMax;

public class Intake extends Submodule {
    private Intake() {
    }

    private static Intake instance = null;

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    private enum ControlState {
        OPEN_LOOP, CLOSED_LOOP
    }

    private ControlState mControlState = ControlState.OPEN_LOOP;

    private double mPercentOut = 0.0;
    private double mDesiredPosition = 0.0;

    private final LazyCANSparkMax mMotor = new LazyCANSparkMax(IntakeConstants.ID, MotorType.kBrushless);
    private final RelativeEncoder mEncoder = mMotor.getEncoder();
    private boolean inOpenLoop = false;

    private final SparkMaxPIDController mPIDController = mMotor.getPIDController();
    private double finalTarget = 0.0;

    @Override
    public void onInit() {
        mMotor.restoreFactoryDefaults();
        configIntakeSparkMax();
        // zero();
    }

    @Override
    public void onStart(double timestamp) {
    }

    @Override
    public void update(double timestamp) {

    }

    @Override
    public void run() {

        if (mControlState == ControlState.OPEN_LOOP) {
            mMotor.set(mPercentOut);
        } else if (mControlState == ControlState.CLOSED_LOOP) {
            mPIDController.setReference(
                    mDesiredPosition,
                    ControlType.kPosition,
                    IntakeConstants.PID_SLOT,
                    0,
                    ArbFFUnits.kPercentOut);
        }
    }

    @Override
    public void stop() {
        mMotor.stopMotor();
    }

    @Override
    public void zero() {
        mEncoder.setPosition(0);
    }

    public void setPercentSpeed(double speed) {
        mControlState = ControlState.OPEN_LOOP;
        mPercentOut = speed;
        inOpenLoop = true;
        setFinalTarget();
    }

    
    public boolean getOpenLoop() {
        return inOpenLoop;
    }

    public void noOpenLoop() {
        inOpenLoop = false;
    }

    public void setFinalTarget() {
        finalTarget = mEncoder.getPosition();
    }

    public void setDesiredPosition(double position) {
        mControlState = ControlState.CLOSED_LOOP;
        mDesiredPosition = position;
    }

    public double getPosition() {
        return mEncoder.getPosition();
    }

    public double getFinalTarget() {
        return finalTarget;
    }

    private void configIntakeSparkMax() {
        mMotor.restoreFactoryDefaults();
        mMotor.setIdleMode(IdleMode.kBrake);
        mMotor.setInverted(IntakeConstants.INVERSION);
        mMotor.setSmartCurrentLimit(IntakeConstants.CURRENT_LIMIT);
        mMotor.enableVoltageCompensation(Constants.VOLTAGE_COMP);

        mPIDController.setFeedbackDevice(mEncoder);
        mPIDController.setFF(IntakeConstants.KF, IntakeConstants.PID_SLOT);
        mPIDController.setP(IntakeConstants.KP, IntakeConstants.PID_SLOT);
        mPIDController.setI(IntakeConstants.KI, IntakeConstants.PID_SLOT);
        mPIDController.setD(IntakeConstants.KD, IntakeConstants.PID_SLOT);

        mPIDController.setPositionPIDWrappingEnabled(true);
        mPIDController.setPositionPIDWrappingMinInput(IntakeConstants.PID_WRAPPING_MIN);
        mPIDController.setPositionPIDWrappingMaxInput(IntakeConstants.PID_WRAPPING_MAX);
    }

}
