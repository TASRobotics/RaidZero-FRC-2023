package raidzero.robot.utils;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Iterator;
import java.util.ListIterator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import pabeles.concurrency.IntOperatorTask.Min;
import raidzero.robot.Constants.ArmConstants;
import raidzero.robot.submodules.Arm;
import raidzero.robot.submodules.Wrist;
import raidzero.robot.submodules.Arm.TargetPosition;

public class ArmPurePursuit {
    
    private Arm arm;
    private double smartMotionConstraints[] = new double[4];
    private ArrayList<Translation2d> waypoints;
    private ArrayList<Translation2d> movements;
    private Iterator<Translation2d> onWaypoint;
    private Iterator<Translation2d> onMovement;
    private Translation2d waypoint;
    private Translation2d movement;
    private Translation2d anglespeeds;
    Translation2d targetPosition;

    private static ArmPurePursuit instance = null;

    /* The purpose of this class is to allow the arm to pursue a more 
     * natural trajectory.  Targets and waypoints are inputted into the class
     * and a trajectory is created based on the current arm conditions.
     */
    private ArmPurePursuit(){
        arm = Arm.getInstance();
    }

    public static ArmPurePursuit getInstance(){
        if (instance == null){
            instance = new ArmPurePursuit();
        }
        return instance;
    }


    
    /**
     * Function that generates waypoints given the wanted waypoints and the
     * current position of the arm.  Checks for self collisions below bot plane
     * and if end location is on opposite side of starting location.  Removes
     * waypoints that are further from the distal than the latter waypoints
     * so no backtracking is done.
     * @param waypoints Waypoints for the arm to follow.
     */
    public void makeWayPoints(Translation2d[] waypoints){
        ArrayList<Translation2d> temp = new ArrayList<Translation2d>();
        temp.add(arm.getState()[1].getTranslation());
        
        Collections.addAll(temp, waypoints);
        ListIterator<Translation2d> checkDistance = temp.listIterator(temp.size());
        Translation2d currentPoint = checkDistance.previous();
        while(checkDistance.hasPrevious()){
            Translation2d previousPoint = checkDistance.previous();
            if (arm.getState()[1].getTranslation().getDistance(currentPoint) <
                previousPoint.getDistance(currentPoint)){
                checkDistance.remove();
            } else {
                currentPoint = previousPoint;
            }
        }
        if(arm.getState()[1].getY()<0.1) temp.add(1, new Translation2d(arm.getState()[1].getX(), 0.1));
        if(Math.signum(waypoints[waypoints.length-1].getX())!=Math.signum(arm.getState()[1].getX()))
            temp.add(1, new Translation2d());
    

        this.waypoints = temp;
        this.movements = new ArrayList<Translation2d>();
        for(int waypointNum = 1; waypointNum<this.waypoints.size();waypointNum++){
            movements.add(this.waypoints.get(waypointNum)
                .minus(this.waypoints.get(waypointNum-1)));
        }
        onMovement = this.movements.iterator();
        onWaypoint = this.waypoints.iterator();
        movement = onMovement.next();
        waypoint = onWaypoint.next();

    }

    /**
     * Changes the speeds of the proximal and distal given the Jacobian has been
     * called in the same cycle.
     */
    public void updateSpeeds(){
        // Use the jacobian for normalized speeds and multiply by the max speed allowed for the arms.
        calculateSpeeds();
        arm.configSmartMotionConstraints(
            smartMotionConstraints[0],
            smartMotionConstraints[1],
            smartMotionConstraints[2],
            smartMotionConstraints[3]);
    }

    private void calculateSpeeds(){
        smartMotionConstraints = anglespeeds.getNorm()>0.01? new double[]{
            smartMotionConstraints[0] = Math.abs(anglespeeds.getX())/anglespeeds.getNorm() *ArmConstants.STRAIGHT_MAX_VEL,
            smartMotionConstraints[1] = Math.abs(anglespeeds.getX())/anglespeeds.getNorm() *ArmConstants.STRAIGHT_MAX_ACCEL,
            smartMotionConstraints[2] = Math.abs(anglespeeds.getY())/anglespeeds.getNorm() *ArmConstants.STRAIGHT_MAX_VEL,
            smartMotionConstraints[3] = Math.abs(anglespeeds.getY())/anglespeeds.getNorm() *ArmConstants.STRAIGHT_MAX_ACCEL,
            }: new double[]{
            ArmConstants.STRAIGHT_MAX_VEL,
            ArmConstants.STRAIGHT_MAX_ACCEL,
            ArmConstants.STRAIGHT_MAX_VEL,
            ArmConstants.STRAIGHT_MAX_ACCEL
        };
    }

    /**
     * Calculates the Jacobian matrix and updated speeds of the arm given the
     * current position and target trajectory to guarantee the motion of the
     * distal is linear in space.
     * @param targetPosition Position that the distal is supposed to go to.
     */

    private void calculateJacobian(){
        Translation2d traj = targetPosition.minus(arm.getState()[1].getTranslation());
        Translation2d relativeUpper = arm.getState()[1].getTranslation().minus(arm.getState()[0].getTranslation());
        traj = traj.div(traj.getNorm());

        // This is the jacobian.  However the form is put in a simplified manner for
        // less calculations, using the positions of the proximal and (relative) distal
        // as placeholders for the trigonometric values.
        anglespeeds = new Translation2d(
            traj.getX()*arm.getState()[0].getY()
            + traj.getY()*arm.getState()[0].getX(),
            - traj.getX()*relativeUpper.getY()
            + traj.getY()*relativeUpper.getX());
    

    }

    private void pursueTrajectory(){
        if(arm.getState()[1].getTranslation().minus(waypoint).getNorm()<ArmConstants.LOOK_AHEAD_DISTANCE
            && onWaypoint.hasNext()) {
            waypoint = onWaypoint.next();
            movement = onMovement.next();
        }
        calculateTarget();
        arm.moveToPoint(new double[]{targetPosition.getX(),targetPosition.getY()});
        updateSpeeds();
    }

    // private boolean calculateAndMoveToTarget() {
    //     // int stageadd = nextStage ? 1 : 0;
    //     Translation2d origin = arm.getState()[1].getTranslation().minus(waypoint);
    //     Translation2d targetLocation;
    //     double scale = (origin.getX()*movement.getX()
    //         +origin.getX()*movement.getX())/movement.getNorm();
    //     double perpdistance = movement.times(scale).minus(origin).getNorm();

    //     if (perpdistance > ArmConstants.LOOK_AHEAD_DISTANCE) return false;
    //     else if (arm.getState()[1].getTranslation().minus(waypoint).getNorm() < ArmConstants.LOOK_AHEAD_DISTANCE){
    //         targetLocation = waypoint;
    //     } else {
    //     Translation2d perpLocation = movement.times(scale).plus(waypoint);
    //     targetLocation = perpLocation.plus(movement
    //         .times(Math.sqrt(Math.pow(ArmConstants.LOOK_AHEAD_DISTANCE, 2)-Math.pow(perpdistance, 2))));
    //     }
    //     calculateJacobian(targetLocation);
    //     updateSpeeds();
        
    //     arm.moveToPoint(new double[]{targetLocation.getX(),targetLocation.getY(),0.0});
    //     return true;
    // }

    private void calculateTarget(){
        Translation2d parallelNorm = movement.div(movement.getNorm());
        Translation2d perpNorm = new Translation2d(1.0, parallelNorm.getAngle().plus(new Rotation2d(Math.PI/2)));
        Translation2d difference = waypoint.minus(arm.getState()[1].getTranslation());
        double perpParam = perpNorm.getX()*difference.getX()+perpNorm.getY()*difference.getY();
        perpParam = perpParam<ArmConstants.LOOK_AHEAD_DISTANCE ? perpParam : ArmConstants.LOOK_AHEAD_DISTANCE-.001;
        double paraParam = Math.signum(parallelNorm.getX()*difference.getX()+parallelNorm.getY()*difference.getY());
            Math.sqrt(Math.pow(ArmConstants.LOOK_AHEAD_DISTANCE, 2)+Math.pow(perpParam, 2));
        targetPosition = parallelNorm.times(paraParam).plus(perpNorm.times(perpParam)).plus(arm.getState()[1].getTranslation());
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
