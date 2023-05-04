package raidzero.robot.utils;

import java.util.Iterator;
import java.util.LinkedList;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class ParallelPoseAdding implements Runnable{

    private LinkedList<VisionPose2d> visionRobotPoses;


    public ParallelPoseAdding(LinkedList<VisionPose2d> visionRobotPoses) {
        this.visionRobotPoses = visionRobotPoses;
    }

    @Override
    public void run(){
        PoseHelper.addPoses(visionRobotPoses);
    }
}
