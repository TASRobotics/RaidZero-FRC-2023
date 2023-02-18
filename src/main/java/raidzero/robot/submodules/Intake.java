package raidzero.robot.submodules;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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

    public enum GamePiece {
        CUBE, CONE
    }

    private double mPercentOut = 0.0;

    private final LazyCANSparkMax mMotor = new LazyCANSparkMax(IntakeConstants.ID, MotorType.kBrushless);

    @Override
    public void onInit() {
        mMotor.restoreFactoryDefaults();
        configIntakeSparkMax();
        mMotor.burnFlash();
    }

    @Override
    public void onStart(double timestamp) {
    }

    @Override
    public void update(double timestamp) {
    }

    @Override
    public void run() {
        mMotor.set(mPercentOut);
    }

    @Override
    public void stop() {
        mMotor.stopMotor();
    }

    @Override
    public void zero() {
    }

    /**
     * Runs intake
     * 
     * @param gamePiece game piece type
     * @param speed     intake speed
     */
    public void intake(GamePiece gamePiece, double speed) {
        if (gamePiece == GamePiece.CUBE) {
            mPercentOut = speed;
        } else if (gamePiece == GamePiece.CONE) {
            mPercentOut = -speed;
        }
    }

    /**
     * Set percent output of intake
     * 
     * @param speed percent output
     */
    public void set(double speed) {
        mPercentOut = speed;
    }

    private void configIntakeSparkMax() {
        mMotor.setIdleMode(IdleMode.kBrake);
        mMotor.setInverted(IntakeConstants.INVERSION);
        mMotor.setSmartCurrentLimit(IntakeConstants.CURRENT_LIMIT);
        mMotor.enableVoltageCompensation(Constants.VOLTAGE_COMP);
    }
}
