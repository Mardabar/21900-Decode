package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ShootSystem {

    private String mode;

    private DcMotorEx ls;
    private DcMotorEx rs;
    private DcMotorEx belt;
    private DcMotorEx elbow;

    private CRServo br;
    private CRServo bl;
    private Servo blocker;
    private CRServo ascension;

    // CONSTANTS

    private final double OVERSHOOT_VEL_MULT = 1.628;
    private final double OVERSHOOT_ANG_MULT = 1;
    private final double ANGLE_CONST = 2.08833333;
    private final int ELBOW_GEAR_RATIO = 28;
    private final double MAX_HEIGHT = 1.4;

        // PID

    private final double lks = 0.0004;
    private final double rks = 0.0004;
    private final double kv = 2800;
    private final double lkp = 0.002;
    private final double rkp = 0.002;

    // SHOOT VARS

    public boolean shootPrep;
    public boolean shootReady;
    private double shootAngle;
    private double shootVel;

    public void valTuner(double lks, double rks){
        ls.setPower(lks);
        rs.setPower(rks);
    }

    // INIT

    public ShootSystem(HardwareMap hardwareMap){

    }

    public void Shoot(){
        //InitShooting();
    }

    // MAIN METHODS

    public void InitShooting(LLResult pic){
        for (LLResultTypes.FiducialResult res : pic.getFiducialResults())
            CheckID(res.getFiducialId());
    }

    private void CheckID(int id){
        /*boolean idCheck = id == 20 || id == 24;
        if (idCheck) {
            double angle = 25.2 + res.getTargetYDegrees();
            double tagDist = (0.646 / Math.tan(Math.toRadians(angle)));
            if (tagDist - 2 < 0)
                tagDist += (tagDist - 2) * 0.28;
            else
                tagDist += (tagDist - 2) * 0.19;

            setShootPos(tagDist);
            shootPrep = true;
        }*/
    }

    private void setShootPos(double dist){
        /* dist is the total distance the ball will travel until it hits the ground
           it's multiplied by 1.3 because the ball will hit the goal first, so using the
           equation, it'll be about 1 meter high (the height of the goal) when it hit our requested distance
         */
        dist *= 1.3;

        // The angle and velocity are both calculated using the distance we found
        shootAngle = ((distToAngle(dist) * OVERSHOOT_ANG_MULT) - 56);
        shootVel = angleToVel(distToAngle(dist)) * OVERSHOOT_VEL_MULT;

        shootPrep = false;
        shootReady = true;
    }

    private void setShootPos(double ix, double iy, double fx, double fy){
        /* dist is the total distance the ball will travel until it hits the ground
           it's multiplied by 1.3 because the ball will hit the goal first, so using the
           equation, it'll be about 1 meter high (the height of the goal) when it hit our requested distance
         */
        double dist = (Math.sqrt(Math.pow(fx - ix, 2) + Math.pow(fy - iy, 2)) / 40) * 1.3;

        // The angle and velocity are both calculated using the distance we found
        shootAngle = ((distToAngle(dist) * OVERSHOOT_ANG_MULT) - 56);
        shootVel = angleToVel(distToAngle(dist)) * OVERSHOOT_VEL_MULT;
    }

    private void setElbowTarget(double angle){
        elbow.setTargetPosition((int) angle);
        elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void runBelt(double speed) {
        belt.setPower(speed);
        br.setPower(speed);
        bl.setPower(-speed);
    }

    // GETTERS

    public double getAngleEnc(){
        return angleToEncoder(shootAngle);
    }

    public double getShootVel(){
        return velToRot(shootVel);
    }

    public double getShootAngle(){
        return shootAngle;
    }

    // CONVERSIONS

    public double distToAngle(double dist){
        return Math.toDegrees(Math.atan(54.88 / (9.8 * dist)));
    }

    // This function translates angle to velocity using the already set maximum height
    public double angleToVel(double angle){
        return Math.sqrt((MAX_HEIGHT * 19.6) / Math.pow(Math.sin(Math.toRadians(angle)), 2));
    }

    // This function translates velocity to motor power specifically for 6000 RPM motors combined with 72 mm Gecko Wheels
    public double velToRot(double vel){
        return vel / (7.2 * Math.PI);
    }

    // This function translates an angle in degrees to an encoder value on 223 RPM motors
    public double angleToEncoder(double angle){
        return angle * ANGLE_CONST * ELBOW_GEAR_RATIO;
    }
}
