package raidzero.robot.auto.actions;

import raidzero.robot.submodules.Swerve;

import raidzero.robot.Constants;
import raidzero.robot.Constants.SwerveConstants;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AutoBalance implements Action {

    private static final Swerve swerve = Swerve.getInstance();
    
    public AutoBalance(){
    }
    
    @Override
    public boolean isFinished() {
        return Math.abs(swerve.getAutoBalancePitch()) < 1;
    }

    @Override
    public void start() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' started!");
    }

    @Override
    public void update() {
        swerve.autoBalance();
    }

    @Override
    public void done() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' finished!");
    }
}
