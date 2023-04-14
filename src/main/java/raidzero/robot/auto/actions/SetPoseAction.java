package raidzero.robot.auto.actions;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import raidzero.robot.submodules.Swerve;

/**
 * Action for following a path.
 */
public class SetPoseAction implements Action {

    private static final Swerve swerve = Swerve.getInstance();
    private Transform2d transform;
    private Pose2d current;
    private Timer timer = new Timer();

    public SetPoseAction(Pose2d c, Transform2d t) {
        transform = t;
        current = c;
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void start() {
        timer.reset();
        timer.start();
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' started!");
    }

    @Override
    public void update() {
        swerve.addVisionMeasurement(current.transformBy(transform.div(10*(timer.get() + 1))), Timer.getFPGATimestamp(),
                new MatBuilder<N3, N1>(Nat.N3(), Nat.N1()).fill(
                        0.01 / (10*(timer.get() + 1)),
                        0.01 / (10*(timer.get() + 1)), 1.0));
    }

    @Override
    public void done() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' finished!");
        timer.stop();
    }
}
