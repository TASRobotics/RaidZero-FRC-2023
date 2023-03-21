package raidzero.robot.submodules;

import com.ctre.phoenix.led.CANdle;

import raidzero.robot.Constants.LightsConstants;

public class Lights extends Submodule {
    private Lights() {}

    private static Lights instance = null;

    public static Lights getInstance() {
        if (instance == null) {
            instance = new Lights();
        }
        return instance;
    }

    private final CANdle candle = new CANdle(LightsConstants.CANDLE_ID);

    private int r, g, b;

    @Override
    public void onInit() {
        candle.configFactoryDefault();
        candle.configLOSBehavior(LightsConstants.LOS_BEHAVIOR);
        candle.configLEDType(LightsConstants.LED_STRIP_TYPE);
        candle.configBrightnessScalar(LightsConstants.BRIGHTNESS_SCALAR);
        candle.configStatusLedState(LightsConstants.STATUS_LED_CONFIG);
        candle.configVBatOutput(LightsConstants.V_BAT_OUTPUT_MODE);
    }

    @Override
    public void onStart(double timestamp) {
        stop();
    }

    @Override
    public void update(double timestamp) {}

    @Override
    public void run() {
        candle.setLEDs(r, g, b);
    }

    @Override
    public void stop() {
        candle.setLEDs(0, 0, 0);
    }

    @Override
    public void zero() {}

    public void setColor(int r, int g, int b) {
        this.r = r;
        this.g = g; 
        this.b = b;
    }
}
