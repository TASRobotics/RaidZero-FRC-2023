package raidzero.robot.submodules;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation;

import edu.wpi.first.wpilibj.Timer;
import raidzero.robot.Constants.LightsConstants;

public class Lights extends Submodule {
    private enum ControlState {
        PLAIN, ANIMATION
    };

    private Lights() {}

    private static Lights instance = null;

    public static Lights getInstance() {
        if (instance == null) {
            instance = new Lights();
        }
        return instance;
    }

    private final CANdle mCANdle = new CANdle(LightsConstants.CANDLE_ID);
    private final Timer mTimer = new Timer();

    private int mR, mG, mB;
    private Animation mAnimation = new StrobeAnimation(0, 0, 0);
    private double mBrightness = 1.0;
    private boolean mFirstTimeNotDetected = false;
    private ControlState mControlState = ControlState.PLAIN;
    private boolean justCalledAnimation = false;

    @Override
    public void onInit() {
        mCANdle.configFactoryDefault();
        mCANdle.configLOSBehavior(LightsConstants.LOS_BEHAVIOR);
        mCANdle.configLEDType(LightsConstants.LED_STRIP_TYPE);
        mCANdle.configBrightnessScalar(LightsConstants.BRIGHTNESS_SCALAR);
        mCANdle.configStatusLedState(LightsConstants.STATUS_LED_CONFIG);
        mCANdle.configVBatOutput(LightsConstants.V_BAT_OUTPUT_MODE);
    }

    @Override
    public void onStart(double timestamp) {
        stop();
    }

    @Override
    public void update(double timestamp) {}

    @Override
    public void run() {
        mCANdle.configBrightnessScalar(mBrightness);
        if(mControlState == ControlState.PLAIN) {
            mCANdle.setLEDs(mR, mG, mB, 0, 8, 512);
        } else if(mControlState == ControlState.ANIMATION) {
            mCANdle.animate(mAnimation);
        }
    }

    @Override
    public void stop() {
        mCANdle.setLEDs(0, 0, 0);
    }

    @Override
    public void zero() {}

    public void setColor(int r, int g, int b) {
        mControlState = ControlState.PLAIN;
        mR = r;
        mG = g; 
        mB = b;
    }

    public void setAnimation(Animation animation) {
        mControlState = ControlState.ANIMATION;
        mAnimation = animation;
    }

    public void setBrightness(double num) {
        mBrightness = num; 
    }

    public void intake(double intake, double deadband) {
        setBrightness(1.0);
        if(intake > deadband) {
            setColor(255, 255, 0);
            justCalledAnimation = false;
        } else if(intake < -deadband) {
            setColor(255, 0, 255);
            justCalledAnimation = false;
        } else if(justCalledAnimation) {
            justCalledAnimation = true;
            // Animation animation = new StrobeAnimation(255, 165, 0, 0, 0.5, -1, 8);
            // Animation animation = new LarsonAnimation(255, 0, 0);
            // Animation animation = new TwinkleOffAnimation(0, 255, 0, 0, 0.1, 512, TwinkleOffAnimation.TwinkleOffPercent.Percent18, 8);    
            Animation animation = new RainbowAnimation(1, 1, -1, false, 8);
            setAnimation(animation);
        }
    }

    public void apples(boolean isDetected) {
        if(isDetected) {
            setBrightness(1.0);
            setColor(0, 255, 0);
            mFirstTimeNotDetected = false;
            return;
        } 
        if(!mFirstTimeNotDetected) {
            mFirstTimeNotDetected = true;
            mTimer.restart();
        }
        if(mTimer.get() > 1) {
            setBrightness(1.0);
            Animation animation = new StrobeAnimation(255, 0, 0, 0, 0.5, -1, 8);
            setAnimation(animation);
        } else {
            setBrightness(mTimer.get());
            setColor(0, 255, 0);
        }
    }
}
