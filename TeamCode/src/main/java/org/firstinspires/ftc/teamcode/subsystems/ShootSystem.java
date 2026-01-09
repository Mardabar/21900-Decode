package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ShootSystem {


    private boolean isFeederUp = false;
    private final double feedDown = 0.0;
    private final double feedUp = 0.3;
    private double anglePos = 0.5;
    private final Telemetry telemetry;
    public Limelight3A cam;

    public DcMotorEx belt;
    public DcMotorEx flywheel;
    private DcMotorEx turretMotor;
    public final Servo angleAdjuster;
    public Servo feeder;

    // CONSTANTS

    private final double OVERSHOOT_VEL_MULT = 1.618;
    private final double OVERSHOOT_ANG_MULT = 1;
    private final double ANGLE_CONST = 2.08833333;
    private final int ELBOW_GEAR_RATIO = 28;
    private final double MAX_HEIGHT = 1.4;

    private final double REST_POS = 0;
    private final double FEED_POS = 0.2;

    // SHOOT VARS

    private double shootAngle;
    private double shootVel;

    private boolean flipped;

    // INIT

    public ShootSystem(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;

        belt = hardwareMap.get(DcMotorEx.class, "belt");
        flywheel = hardwareMap.get(DcMotorEx.class, "cannon");
        angleAdjuster = hardwareMap.get(Servo.class, "angleServo");
        feeder = hardwareMap.get(Servo.class, "feeder");

        flywheel.setDirection(DcMotorEx.Direction.FORWARD);
        belt.setDirection(DcMotorEx.Direction.FORWARD);

        belt.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //angleAdjuster.scaleRange(0, 1);
        //feeder.scaleRange(REST_POS, FEED_POS);

        angleAdjuster.setPosition(0.5);
        feeder.setPosition(0.0);

        cam = hardwareMap.get(Limelight3A.class, "limelight");
        cam.pipelineSwitch(0);
    }

    // PUBLIC METHODS

    public void StopMotors(){
        flywheel.setVelocity(0);
        RunBelt(0);
    }

    public void Shoot(){
        UpdatePositions(cam.getLatestResult());
        double velocity = velToPow(shootVel);
        flywheel.setVelocity(velocity);
        setShootAngle(shootAngle);


        boolean deltaConditions = flywheel.getVelocity() > velDelta(velocity, -1) && flywheel.getVelocity() < velDelta(velocity, 1);
        if (deltaConditions)
            RunBelt(1);
        else
            RunBelt(0);
    }

    private double velDelta(double vel, double sign){
        return vel + (sign * 15);
    }

    public void UseFeeder() {
//        if (feeder.getPosition() < 0.99 && !flipped)
//            feeder.setPosition(1);
//        else if (!flipped){
//            flipped = true;
//            feeder.setPosition(0);
//        }
//        else if (feeder.getPosition() < 0.01 && flipped)
//            flipped = false;


        if (feeder.getPosition() < 0.1) {
            feeder.setPosition(feedUp);
        } else {
            feeder.setPosition(feedDown);
        }

        //feeder.setPosition(1);

    }



    /// BLAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAT BIM BAM BOOM BOOM FLIPPER MANUEL CONTROL
    public void toggleFeeder() {
        if (isFeederUp) {
            feeder.setPosition(feedDown); // Go to 0.0
            isFeederUp = false;
        } else {
            feeder.setPosition(feedUp);   // Go to 0.6
            isFeederUp = true;
        }

    }

    /// uhhhhh manual stick input for servo cause fuck me
    public void moveAngleManual(double joystickInput) {

        double speed = 0.001;

        anglePos += (joystickInput * speed);

        // makes sure the value stays in between 0 and 1, maybe
        //  ¯\_(ツ)_/¯
        anglePos = Math.clamp(anglePos, 0, 1);

        angleAdjuster.setPosition(anglePos);

        telemetry.addData("Angle Manual Pos", anglePos);
    }

    // MAIN METHODS


    public void blatFuck(double anglePower, boolean runFeeder) {

        angleAdjuster.setPosition(anglePower);

        if (runFeeder) {
            feeder.setPosition(feedUp);
        } else {
            feeder.setPosition(feedDown);
        }

        telemetry.addData("Target Angle Pos", anglePower);

    }


    public void fuckServos(){

        if (feeder.getPosition() < 0.1) {
            feeder.setPosition(feedUp);
        } else {
            feeder.setPosition(feedDown);
        }
    }




    private void UpdatePositions(LLResult pic){
        for (LLResultTypes.FiducialResult res : pic.getFiducialResults())
            CheckID(res, res.getFiducialId());
    }

    private void CheckID(LLResultTypes.FiducialResult res, int id){
        boolean idCheck = id == 20 || id == 24;
        if (idCheck)
            UpdateVars(res);
    }

    private void UpdateVars(LLResultTypes.FiducialResult res){
        double angle = 25.2 + res.getTargetYDegrees();
        double tagDist = (0.646 / Math.tan(Math.toRadians(angle)));

        if (tagDist - 2 < 0)
            tagDist += (tagDist - 2) * 0.28;
        else
            tagDist += (tagDist - 2) * 0.19;

        setShootPos(tagDist);
    }

    private void setShootPos(double dist){
        /* dist is the total distance the ball will travel until it hits the ground
           it's multiplied by 1.3 because the ball will hit the goal first, so using the
           equation, it'll be about 1 meter high (the height of the goal) when it hit our requested distance
         */
        dist *= 1.3;

        // The angle and velocity are both calculated using the distance we found
        shootAngle = ((distToAngle(dist) * OVERSHOOT_ANG_MULT) - 45);
        shootVel = angleToVel(distToAngle(dist)) * OVERSHOOT_VEL_MULT;
    }

    private void setShootAngle(double angle){
        angleAdjuster.setPosition(Math.clamp(angle / 300, 0, 1));
    }

    public void RunBelt(double speed) {
        belt.setPower(speed);
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
    public double velToPow(double vel) {
        return (vel / (9.6 * Math.PI)) * 2800;
    }
}
