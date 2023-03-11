package raidzero.robot.auto.actions;

import raidzero.robot.submodules.Swerve;

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