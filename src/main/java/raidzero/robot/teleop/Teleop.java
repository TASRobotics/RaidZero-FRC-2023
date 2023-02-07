package raidzero.robot.teleop;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import raidzero.robot.submodules.Arm;
import raidzero.robot.utils.JoystickUtils;
import edu.wpi.first.math.MathUtil;


public class Teleop {

    private static Teleop instance = null;
    private static XboxController p1 = new XboxController(0);
    private static XboxController p2 = new XboxController(1);

    private static final Arm arm = Arm.getInstance();
    private double rampRate = 0.0;

    public static Teleop getInstance() {
        if (instance == null) {
            instance = new Teleop();
        }
        return instance;
    }

    public void onStart() {
    }

    /**
     * Continuously loops in teleop.
     */
    public void onLoop() {
        /**
         * shared controls
         */

        /**
         * p1 controls
         */
        p1Loop(p1);
        /**
         * p2 controls
         */
        p2Loop(p2);
    }

    private int mode = 0;
    private double[] coord = {-1.7, 0.3};
    private double a1 = 1.0;
    private double a2 = 1.0;
    private double s1_q1 = 0;
    private double s1_q2 = 0;
    private double s2_q1 = 0;
    private double s2_q2 = 0;
    private double q1 = 0;
    private double q2 = 0;
    private double radius = 0;
    private double radius_sq = 0;
    private double theta = 0;
    private double acosarg = 0;
    private double elbow_supplement = 0;
    private double alpha = 0;

    private void p1Loop(XboxController p) {

        rampRate = SmartDashboard.getNumber("Ramp Rate", 0);
        SmartDashboard.putNumber("Ramp Rate", rampRate);
        SmartDashboard.putNumber("x", p.getRightX());
        SmartDashboard.putNumber("q1", q1);
        SmartDashboard.putNumber("q2", q2);
        SmartDashboard.putNumber("x", coord[0]);
        SmartDashboard.putNumber("y", coord[1]);

        //arm.setArmRampRate(rampRate);

        if (p.getAButtonPressed()) {
            mode = 1;

        } else if (p.getBButtonPressed()) {
            mode = 2;

        } else if (p.getStartButtonPressed()){
            mode = 3;
        }

        if (mode == 1) {
            arm.moveArm(p.getRightX() * 0.2, p.getLeftX() * 0.2);
        } else if (mode == 2) {
            if(p.getYButtonPressed()){
                    arm.moveToAngle(52, -30);
            }else if(p.getXButtonPressed()){
                    arm.moveToAngle(-52,30);
            }else if(p.getBackButtonPressed()){
                    arm.moveToAngle(0,0);
            }
        } else if (mode == 3) {

            if (!(Math.abs(coord[0]) >= 1.75) && !(coord[1] <= 0.1) && !(coord[1] >= 1.75)){
                coord[0] += MathUtil.applyDeadband(p.getRightX()*0.07, 0.05);
                coord[1] += MathUtil.applyDeadband(p.getLeftY()*-0.07, 0.05);
            }
            if (p.getLeftBumperPressed()){
                coord[0] = -1.7;
                coord[1] = 0.3; 
            }

            //position of point 
            radius_sq = coord[0]*coord[0] + coord[1]*coord[1];
            radius = Math.sqrt(radius_sq);
            //angle of target point
            theta = Math.atan2(coord[0], -1*coord[1]);

            //use law of cosines to compute the elbow angle
            acosarg = (radius_sq - a1*a1 - a2*a2)/(-2 * a1 * a2);
            if (acosarg<-1.0) elbow_supplement = Math.PI;
            else if (acosarg>1.0) elbow_supplement = 0.0;
            else elbow_supplement = Math.acos(acosarg);

            //use law of sines to find the angle at the bottom vertex of the triangle defined by the links
            if (radius>0.0) alpha = Math.asin(a2 * Math.sin(elbow_supplement) / radius);
            else alpha = 0.0;

            //compute the two solutions with opposite elbow sign
            s1_q1 = Math.toDegrees(theta - alpha);
            s1_q2 = Math.toDegrees(Math.PI - elbow_supplement);
            
            s2_q1 = Math.toDegrees(theta + alpha);
            s2_q2 = Math.toDegrees(elbow_supplement - Math.PI);
            
            if (Math.abs(s1_q1)>160 || Math.abs(s1_q2)>160){
                q1 = s2_q1;
                q2 = s2_q2;
            } else{
                q1 = s1_q1;
                q2 = s1_q2;
            }

            // q2 = Math.toDegrees(Math.acos((x*x + y*y - a1*a1 - a2*a2)/(2*a1*a2)));
            // q1 = Math.toDegrees(Math.atan(y/x)-Math.atan((a2*Math.sin(q2))/(a1+a2*Math.cos(q2))));
            arm.moveToAngle(90-Math.abs(q1), -1*(Math.abs(q2)));
            //arm.moveToAngle(Math.signum(q1)*(90-Math.abs(q1)), -1*q2);

        }

    }

    private void p2Loop(XboxController p) {
    }

}
