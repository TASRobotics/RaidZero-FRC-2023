package raidzero.robot.utils;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import raidzero.robot.Constants.ArmConstants;
import raidzero.robot.submodules.Arm;
import raidzero.robot.submodules.Wrist;

public class ArmPurePursuit extends Arm {
    
    private Arm arm ;
    private double smartMotionConstraints[] = new double[4];
    private Translation2d waypoints[];
    private Translation2d movements[];
    private int stage;
    private boolean trajectorySuccess = true;

    private static ArmPurePursuit instance = null;


    private ArmPurePursuit(){
    }

    public static ArmPurePursuit getInstance(){
        if (instance == null){
            instance = new ArmPurePursuit();
        }
        return instance;
    }

    @Override
    public void onInit(){
        super.onInit();
        
    }

    @Override
    public void onStart(double timestamp){
        super.onStart(timestamp);
        makeWayPoints(new Translation2d[]{getState()[1].getTranslation()});
    }

    
    @Override
    public void update(double timestamp){
        super.update(timestamp);
        
        trajectorySuccess = pursueTrajectory();
    }
    public void makeWayPoints(Translation2d[] waypoints){
        stage = 0;
        ArrayList<Translation2d> temp = new ArrayList<Translation2d>();
        temp.add(getState()[1].getTranslation());
        if(getState()[1].getY()<0.0) temp.add(new Translation2d(arm.getState()[1].getX(), 0.1));
        if(Math.signum(waypoints[waypoints.length-1].getX())!=Math.signum(arm.getState()[1].getX()))
            temp.add(new Translation2d());
        Collections.addAll(temp, waypoints);
        this.waypoints = temp.toArray(new Translation2d[temp.size()]);
        this.movements = new Translation2d[waypoints.length-1];
        for(int waypointNum = 1; waypointNum<waypoints.length;waypointNum++){
            movements[waypointNum-1] = waypoints[waypointNum]
                .minus(waypoints[waypointNum-1]);
        }
        System.out.println(this.waypoints.length);

    }

    private void updateSpeeds(){
        arm.configSmartMotionConstraints(
            smartMotionConstraints[0],
            smartMotionConstraints[1],
            smartMotionConstraints[2],
            smartMotionConstraints[3]);
    }

    private void defaultSpeeds(){
        smartMotionConstraints = new double[]{
            ArmConstants.STRAIGHT_MAX_VEL,
            ArmConstants.STRAIGHT_MAX_ACCEL,
            ArmConstants.STRAIGHT_MAX_VEL,
            ArmConstants.STRAIGHT_MAX_ACCEL
        };
    }

    private void calculateJacobian(Translation2d targetPosition){
        Translation2d traj = targetPosition.minus(arm.getState()[1].getTranslation());
        Translation2d relativeUpper = arm.getState()[1].getTranslation().minus(arm.getState()[0].getTranslation());
        traj = traj.div(traj.getNorm());
        // This is the jacobian.  However the form is put in a simplified manner for
        // less calculations, using the positions of the proximal and (relative) distal
        // as placeholders for the trigonometric values.
        Translation2d anglespeeds = new Translation2d(
            traj.getX()*arm.getState()[0].getY()
            + traj.getY()*arm.getState()[0].getX(),
            - traj.getX()*relativeUpper.getY()
            + traj.getY()*relativeUpper.getX());
        
        if (anglespeeds.getNorm()>0.01){
            smartMotionConstraints[0] = anglespeeds.getX()/anglespeeds.getNorm() *ArmConstants.STRAIGHT_MAX_VEL;
            smartMotionConstraints[1] = anglespeeds.getX()/anglespeeds.getNorm() *ArmConstants.STRAIGHT_MAX_ACCEL;
            smartMotionConstraints[2] = anglespeeds.getY()/anglespeeds.getNorm() *ArmConstants.STRAIGHT_MAX_VEL;
            smartMotionConstraints[3] = anglespeeds.getY()/anglespeeds.getNorm() *ArmConstants.STRAIGHT_MAX_ACCEL;
        } else defaultSpeeds();

        
            

    }

    private boolean pursueTrajectory(){
        for(int stagetry = movements.length-1; stagetry> stage; stagetry--)
        if(calculateAndMoveToTarget(stagetry)){
            stage = stagetry;
            return true;
        }
        return (calculateAndMoveToTarget(stage));
    }

    private boolean calculateAndMoveToTarget(int nextStage) {
        // int stageadd = nextStage ? 1 : 0;
        Translation2d origin = arm.getState()[1].getTranslation().minus(waypoints[nextStage]);
        Translation2d targetLocation;
        double scale = (origin.getX()*movements[nextStage].getX()
            +origin.getX()*movements[nextStage].getX())/movements[nextStage].getNorm();
        double perpdistance = movements[nextStage].times(scale).minus(origin).getNorm();

        if (perpdistance > ArmConstants.LOOK_AHEAD_DISTANCE) return false;
        else if (arm.getState()[1].getTranslation().minus(waypoints[waypoints.length-1]).getNorm() < ArmConstants.LOOK_AHEAD_DISTANCE){
            targetLocation = waypoints[waypoints.length-1];
        } else {
        Translation2d perpLocation = movements[nextStage].times(scale).plus(waypoints[nextStage]);
        targetLocation = perpLocation.plus(movements[nextStage]
            .times(Math.sqrt(Math.pow(ArmConstants.LOOK_AHEAD_DISTANCE, 2)-Math.pow(perpdistance, 2))));
        }
        calculateJacobian(targetLocation);
        updateSpeeds();
        
        arm.moveToPoint(new double[]{targetLocation.getX(),targetLocation.getY(),0.0});
        return true;
    }


    

    public void movePurePursuit(TargetPosition endTargets){
        Translation2d wayPoints[];
        switch (endTargets){
            case FLOOR_INTAKE:
            wayPoints = new Translation2d[]{
                new Translation2d(ArmConstants.INTER_FLOOR_INTAKE[0], ArmConstants.INTER_FLOOR_INTAKE[1]),
                new Translation2d(ArmConstants.FLOOR_INTAKE[0], ArmConstants.FLOOR_INTAKE[1])};
                
            case CUBE_GRID_MEDIUM:
                wayPoints = new Translation2d[]{
                    new Translation2d(ArmConstants.INTER_CUBE_GRID_MEDIUM[0], ArmConstants.INTER_CUBE_GRID_MEDIUM[1]),
                    new Translation2d(ArmConstants.CUBE_GRID_MEDIUM[0], ArmConstants.CUBE_GRID_MEDIUM[1])};
                
            case CUBE_GRID_HIGH:
                wayPoints = new Translation2d[]{
                    new Translation2d(ArmConstants.INTER_CUBE_GRID_HIGH[0], ArmConstants.INTER_CUBE_GRID_HIGH[1]),
                    new Translation2d(ArmConstants.CUBE_GRID_HIGH[0], ArmConstants.CUBE_GRID_HIGH[1])};
                
            case HUMAN_PICKUP_STATION:
                wayPoints = new Translation2d[]{
                    new Translation2d(ArmConstants.INTER_HUMAN_PICKUP_STATION[0], ArmConstants.INTER_HUMAN_PICKUP_STATION[1]),
                    new Translation2d(ArmConstants.HUMAN_PICKUP_STATION[0], ArmConstants.HUMAN_PICKUP_STATION[1])};
        
        makeWayPoints(wayPoints);
        }
    }
}
