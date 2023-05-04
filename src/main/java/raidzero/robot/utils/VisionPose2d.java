package raidzero.robot.utils;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import raidzero.robot.Constants.VisionConstants;

public class VisionPose2d extends Pose2d{

    Matrix<N3,N1> visionErrors;
    double timestamp;

    public VisionPose2d(Pose2d pose, double timestamp, double visionErrors){
        super(pose.getTranslation(), pose.getRotation());
        this.timestamp = timestamp;
        this.visionErrors = new MatBuilder<N3,N1>(Nat.N3(), Nat.N1()).fill(
            VisionConstants.DISTANCEERRORFACTOR* visionErrors,
            VisionConstants.DISTANCEERRORFACTOR* visionErrors,
            VisionConstants.ANGLEERRORFACTOR* visionErrors);
    }

    public double getTimestamp(){
        return timestamp;
    }

    public Matrix<N3,N1> getVisionErrors(){
        return visionErrors;
    }
    
}
