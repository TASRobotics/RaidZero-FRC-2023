package raidzero.robot.utils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class ParallelPoseAdding implements Runnable{

    private double[] timestamps;
    private Pose2d[] newRobotPoses;
    private Matrix<N3, N1> errors;


    public ParallelPoseAdding(Pose2d[] newRobotPoses, double[] timestamps, Matrix<N3, N1> errors) {
        this.newRobotPoses = newRobotPoses;
        this.timestamps = timestamps;
        this.errors = errors;
    }

    @Override
    public void run(){
        PoseHelper.addPoses(newRobotPoses, timestamps, errors);
    }
}
