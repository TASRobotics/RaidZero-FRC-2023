package raidzero.robot.submodules;

import java.util.stream.IntStream;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.ml.LogisticRegression;
import org.opencv.ml.Ml;
import org.opencv.ml.TrainData;

import java.util.stream.IntStream;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.ml.LogisticRegression;
import org.opencv.ml.Ml;
import org.opencv.ml.TrainData;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import raidzero.robot.Constants;
import raidzero.robot.Constants.WristConstants;
import raidzero.robot.wrappers.LazyCANSparkMax;

public class Wrist extends Submodule {
    private Wrist() {
    }

    private static Wrist instance = null;

    public static Wrist getInstance() {
        if (instance == null) {
            instance = new Wrist();
        }
        return instance;
    }

    private enum ControlState {
        OPEN_LOOP, CLOSED_LOOP
    }

    
    private ControlState mControlState = ControlState.OPEN_LOOP;

    private double mPercentOut = 0.0;
    private double mDesiredAngle = 0.0;
    private double zeroOffset = 0.0;

    private LogisticRegression limitSwitchModel;
    private Mat limitSwitchTriggerData = new Mat(WristConstants.LIMITSWITCHBUFFERSIZE, 1, CvType.CV_16F);
    private Mat limitSwitchEncoderData = new Mat(WristConstants.LIMITSWITCHBUFFERSIZE, 1, CvType.CV_16F);
    private int indexPosition;
    private ArmFeedforward mFeedforward = new ArmFeedforward(0, 0, 0);

    private final LazyCANSparkMax mMotor = new LazyCANSparkMax(WristConstants.ID, MotorType.kBrushless);

    private final RelativeEncoder mEncoder = mMotor.getEncoder();
    private final SparkMaxLimitSwitch inZoneLimitSwitch = mMotor.getForwardLimitSwitch(Constants.WristConstants.LIMITSWITCHPOLARITY);


    private final SparkMaxPIDController mPIDController = mMotor.getPIDController();

    @Override
    public void onInit() {
        mMotor.restoreFactoryDefaults();
        configWristSparkMax();
        mMotor.burnFlash();
        zero();
        limitSwitchModel.setLearningRate(0.01);
        limitSwitchModel.setIterations(10);
        indexPosition = 0;
    }

    @Override
    public void onStart(double timestamp) {
    }

    @Override
    public void update(double timestamp) {
        limitSwitchTriggerData.put(indexPosition, 1, inZoneLimitSwitch.isPressed() ? 1 : 0);
        limitSwitchEncoderData.put(indexPosition, 1, mMotor.getEncoder().getPosition()/WristConstants.ENCODER_NORMALIZATION);
        indexPosition++;
        indexPosition %= WristConstants.LIMITSWITCHBUFFERSIZE;
        double howBalanced = 0;
        for(int bufferIndex = 0; bufferIndex < WristConstants.LIMITSWITCHBUFFERSIZE; bufferIndex++){
            howBalanced += limitSwitchTriggerData.get(bufferIndex,1)[0]/WristConstants.LIMITSWITCHBUFFERSIZE;
        }
        howBalanced = howBalanced/WristConstants.LIMITSWITCHBUFFERSIZE;
        if (Math.pow(howBalanced-0.5, 2)< 0.2 && timestamp > WristConstants.LIMITSWITCHBUFFERSIZE*Constants.TIMEOUT_S){
            align();
        }
    }

    private void align() {
        limitSwitchModel.train(limitSwitchTriggerData, Ml.COL_SAMPLE, limitSwitchEncoderData);
        Mat thetas = limitSwitchModel.get_learnt_thetas();
        int edge = (int) (thetas.get(0, 0)[0]/thetas.get(1,0)[0]);
        mEncoder.setPosition(mEncoder.getPosition() + edge-WristConstants.LIMITSWITCHOFFSET);
    }

    @Override
    public void run() {
        if (mControlState == ControlState.OPEN_LOOP) {
            mMotor.set(mPercentOut);
        } else if (mControlState == ControlState.CLOSED_LOOP) {
            mPIDController.setReference(
                    mDesiredAngle,
                    ControlType.kSmartMotion,
                    WristConstants.SMART_MOTION_SLOT,
                    mFeedforward.calculate(getAngle().getRadians(), 0),
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

    /**
     * Sets desired percent speed [-1, 1]
     * 
     * @param speed percent speed
     */
    public void setPercentSpeed(double speed) {
        mControlState = ControlState.OPEN_LOOP;
        mPercentOut = speed;
    }

    /**
     * Set desired wrist angle (degrees) [0, 360]
     * 
     * @param angle desired angle
     */
    public void setDesiredAngle(double angle) {
        mControlState = ControlState.CLOSED_LOOP;
        mDesiredAngle = angle;
        mEncoder.getVelocity();
    }

    /**
     * Get current wrist angle
     * 
     * @return current angle
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(mEncoder.getPosition());
    }

    /**
     * Get current wrist velocity
     * 
     * @return current velocity
     */
    public Rotation2d getVelocity() {
        return Rotation2d.fromDegrees(mEncoder.getVelocity());
    }

    private void configWristSparkMax() {
        mMotor.setIdleMode(IdleMode.kBrake);
        mMotor.setInverted(WristConstants.INVERSION);
        mMotor.setSmartCurrentLimit(WristConstants.CURRENT_LIMIT);
        mMotor.enableVoltageCompensation(Constants.VOLTAGE_COMP);

        mEncoder.setInverted(WristConstants.ENCODER_INVERSION);
        mEncoder.setPositionConversionFactor(WristConstants.POSITION_CONVERSION_FACTOR);
        mEncoder.setVelocityConversionFactor(WristConstants.VELOCITY_CONVERSION_FACTOR);

        mPIDController.setFeedbackDevice(mEncoder);
        mPIDController.setFF(WristConstants.KF, WristConstants.SMART_MOTION_SLOT);
        mPIDController.setP(WristConstants.KP, WristConstants.SMART_MOTION_SLOT);
        mPIDController.setI(WristConstants.KI, WristConstants.SMART_MOTION_SLOT);
        mPIDController.setD(WristConstants.KD, WristConstants.SMART_MOTION_SLOT);
        mPIDController.setSmartMotionMinOutputVelocity(WristConstants.MIN_VEL, WristConstants.SMART_MOTION_SLOT);
        mPIDController.setSmartMotionMaxVelocity(WristConstants.MAX_VEL, WristConstants.SMART_MOTION_SLOT);
        mPIDController.setSmartMotionMaxAccel(WristConstants.MAX_ACCEL, WristConstants.SMART_MOTION_SLOT);
        mPIDController.setSmartMotionAllowedClosedLoopError(WristConstants.MIN_ERROR, WristConstants.SMART_MOTION_SLOT);
        mPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, WristConstants.SMART_MOTION_SLOT);
        mPIDController.setPositionPIDWrappingEnabled(true);
        mPIDController.setPositionPIDWrappingMinInput(WristConstants.PID_WRAPPING_MIN);
        mPIDController.setPositionPIDWrappingMaxInput(WristConstants.PID_WRAPPING_MAX);
    }
}
