package raidzero.robot.utils;

import java.util.Iterator;
import java.util.LinkedList;
import java.util.ListIterator;

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
    

    /**Method that adds a collection of poses to the odometery measurements
     * and kalman filter estimator.  Will confirm that all the poses are self
     * consistent with each other within tolerance, otherwise no poses will be
     * added and false will be returned.
     * 
     * @param posesToAdd an array of 2d poses to be added to the drivetrain
     * measurements as a vision pose
     * @param timestamps an array of the timestamps of the 2d poses
     * @param visionMeasurementStdDevs an array of the measurement errors
     * associated with the poses to be added
     * @return Whether the poses were successfully added to the drivetrain
     * measurements
     */
    public static boolean addPoses(LinkedList<VisionPose2d> listPosesToAdd){
        
        
        
        if(listPosesToAdd.isEmpty()) return false;

        // while(poseIterator.hasNext()) 
        //     if(driveTrain.getHistPose(poseIterator.next().getTimestamp())
        //     .isEmpty()) return false;

        // VisionPose2d[] posesToAdd;
        // poseIterator = po
        // // if (timestamps.length==0) return false;
        // // for (double timestamp:timestamps){
        // //     if (driveTrain.getHistPose(timestamp).isEmpty()) return false;
        // // }
        VisionPose2d[] posesToAdd = (VisionPose2d[]) listPosesToAdd.toArray();

        if (!isSelfConsistent(posesToAdd)) return false;
        for(int poseNum = 0; poseNum<posesToAdd.length;poseNum++){
            if (driveTrain.getHistPose(posesToAdd[poseNum].getTimestamp()).isEmpty() ||
                !inTolerance(driveTrain.getHistPose(posesToAdd[poseNum].getTimestamp()).get(), posesToAdd[poseNum])) 
                return false;
        }

        for(int poseNum = 0; poseNum<posesToAdd.length;poseNum++){
            driveTrain.addVisionMeasurement(posesToAdd[poseNum]);
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
