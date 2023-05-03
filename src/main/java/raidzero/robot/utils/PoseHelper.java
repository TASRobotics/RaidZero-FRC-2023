package raidzero.robot.utils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import raidzero.robot.Constants.PoseConstants;
import raidzero.robot.Constants.VisionConstants;
import raidzero.robot.submodules.Swerve;

public class PoseHelper {

    private static Swerve driveTrain = Swerve.getInstance();
    

    public static boolean addPoses(Pose2d[] posesToAdd, double[] timestamps, 
        Matrix<N3, N1> visionMeasurementStdDevs){
        
        if (timestamps.length==0) return false;
        for (double timestamp:timestamps){
            if (driveTrain.getHistPose(timestamp).isEmpty()) return false;
        }
        
        if (!isSelfConsistent(posesToAdd)) return false;
        for(int poseNum = 0; poseNum<posesToAdd.length;poseNum++){
            if (!inTolerance(driveTrain.getHistPose(timestamps[poseNum]).get(), posesToAdd[poseNum])) return false;
        }

        for(int poseNum = 0; poseNum<posesToAdd.length;poseNum++){
            driveTrain.addVisionMeasurement(posesToAdd[poseNum], timestamps[poseNum], visionMeasurementStdDevs);
        }
        return true;
    }

    private static boolean isSelfConsistent(Pose2d[] posesToAdd){
        for(int first = 0; first< posesToAdd.length;first++){
            for(int second = first+1; second <posesToAdd.length; second++){
                if (!inTolerance(posesToAdd[first], posesToAdd[second])) return false;
            }
        }
        return true;
    }

    private static boolean inTolerance(Pose2d initPose, Pose2d finalPose){
        Transform2d difference = finalPose.minus(initPose);
        if (difference.getRotation().getDegrees()>PoseConstants.ANGLE_DIFF_TOLERANCE
                && difference.getTranslation().getNorm()>PoseConstants.DISTANCE_ADD_TOLERANCE) return false;
        return true;

    }
    
}
