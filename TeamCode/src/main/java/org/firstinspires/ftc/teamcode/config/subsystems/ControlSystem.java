package org.firstinspires.ftc.teamcode.config.subsystems;

import java.util.Map;
import java.util.TreeMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ControlSystem {
    // Flywheel PID and Servo Positions
    public static double kP = 0.007, kS = 0.02, kV = 0.00045;
    public static final double openPos = .35, closePos = 0, IDLE_VELO = 300;

    // Quadratic Drag Vars
    private static final double BALL_MASS = 0.0748;
    private static final double GRAVITY = 9.8;
    private static final double DRAG_COEFF = 0.3;

    // Sensor Vars
    private final VoltageSensor battery;
    public final Limelight3A cam;

    // Motor and Servo Vars
    public final Servo angleAdjuster, feeder;
    public final DcMotorEx belt, flywheel;

    // Servo Angle Stuff
    private final TreeMap<Double, Double> angleMap = new TreeMap<>();
    public double anglePos = 0.5, shootVel, beltSpeed = 1, manualServoPos = 0.15;
    public double rawVelocity;

    public ControlSystem(HardwareMap hardwareMap, Telemetry telemetry) {
        belt = hardwareMap.get(DcMotorEx.class, "belt");
        flywheel = hardwareMap.get(DcMotorEx.class, "cannon");
        angleAdjuster = hardwareMap.get(Servo.class, "angleServo");
        feeder = hardwareMap.get(Servo.class, "feeder");
        battery = hardwareMap.voltageSensor.iterator().next();
        cam = hardwareMap.get(Limelight3A.class, "limelight");

        // Motor init
        flywheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel.setDirection(DcMotorEx.Direction.REVERSE);
        belt.setDirection(DcMotorEx.Direction.REVERSE);
        belt.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        initDistances();
        cam.pipelineSwitch(0);
        cam.start();
    }

    private void initDistances() {
        // linear interpolation table (dist in meters to servo pos)
        angleMap.put(0.9144, 0.122);
        angleMap.put(1.0922, 0.132);
        angleMap.put(1.1938, 0.130);
        angleMap.put(1.2954, 0.135);
        angleMap.put(1.5240, 0.150);
        angleMap.put(1.6256, 0.237);
        angleMap.put(2.0574, 0.248);
        angleMap.put(3.0734, 0.19);
    }


    //    kP = 0.007,
    //    kS = 0.02,
    //    kV = 0.00045;


    public void updateFlywheelControl(double targetTPS) {
        double currentTPS = flywheel.getVelocity();
        double ff = (kV * targetTPS) + (kS * Math.signum(targetTPS));
        double fb = kP * (targetTPS - currentTPS);

        double power = (ff + fb) * (12.0 / battery.getVoltage());
        flywheel.setPower(Math.max(-1.0, Math.min(1.0, power)));
    }

    public void Shoot() {
        LLResult result = cam.getLatestResult();
        if (result != null && result.isValid())
            updateVars(result);

        updateFlywheelControl(shootVel);
        angleAdjuster.setPosition(anglePos);
    }



    private void updateVars(LLResult result) {
        for (LLResultTypes.FiducialResult res : result.getFiducialResults())
            if (res.getFiducialId() == 20 || res.getFiducialId() == 24) {
                double angle = res.getTargetYDegrees();
                double limeDist = (0.646 / Math.tan(Math.toRadians(angle))) + 0.2;

                beltSpeed = (limeDist < 2.6) ? 0.65 : 0.3;
                setShootPos(limeDist);
            }
    }

    private void setShootPos(double dist) {
        double distMult = dist * 1;
        double veloMult = 1;

        /*double targetAngle = Math.toDegrees(Math.atan(54.88 / (9.8 * distMult)));
        double rawVel = Math.sqrt((MAX_HEIGHT * 19.6) / Math.pow(Math.sin(Math.toRadians(targetAngle)), 2)) * veloMult;*/

        double terminalVel = (BALL_MASS * GRAVITY) / DRAG_COEFF;
        double rawVel = (distMult * GRAVITY) / (terminalVel * Math.cos(ServoPosToRadians(interpolateAngle(distMult))));
        rawVelocity = rawVel;
        shootVel = ((rawVel * veloMult) / (9.6 * Math.PI)) * 2800; // Vel to TPS
    }

    // LERP (we love lerp) boutta lerp you
    private double interpolateAngle(double distance) {
        Map.Entry<Double, Double> low = angleMap.floorEntry(distance);
        Map.Entry<Double, Double> high = angleMap.ceilingEntry(distance);

        if (low == null && high == null) return 0;
        if (low == null) anglePos = high.getValue();
        else if (high == null) anglePos = low.getValue();
        else
            anglePos = low.getValue() + (distance - low.getKey()) * ((high.getValue() - low.getValue()) / (high.getKey() - low.getKey()));
        anglePos = Math.max(0, Math.min(1, anglePos));
        return anglePos;
    }

    public double ServoPosToRadians(double pos) { return Math.toRadians(90 - (pos * 300)); }

    public void RunBelt(double speed) { belt.setPower(speed); }

    public void stopBelt() { belt.setPower(0); }

    public void StopMotors() {
        flywheel.setPower(0);
        stopBelt();
    }

    public void adjustServoManual(boolean up, boolean down) {
        double increment = 0.001;
        if (up) manualServoPos += increment;
        else if (down) manualServoPos -= increment;
        manualServoPos = Math.clamp(manualServoPos, 0, 1); angleAdjuster.setPosition(manualServoPos);
    }
}