package org.firstinspires.ftc.teamcode.config.subsystems;


import static org.firstinspires.ftc.teamcode.config.pedroPathing.Tuning.follower;


import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;


import java.util.Map;
import java.util.TreeMap;



public class Shooter {

    public final Servo angleAdjuster, feeder;
    public final DcMotorEx belt, flywheel;

    public static double kP = 0.007, kS = 0.02, kV = 0.00045;
    public final double openPos = .35, closePos = 0;
    private static final double MAX_HEIGHT = 1.4;

    private final VoltageSensor battery;
    private final Follower fol;
    public final Limelight3A cam;
    private final Telemetry telemetry;
    private final TreeMap<Double, Double> angleMap = new TreeMap<>();
    public double anglePos = 0.5, shootVel, beltSpeed = 1, manualServoPos = 0.15;


    public Shooter(HardwareMap hardwareMap, Telemetry telemetry){
        this.fol = follower;
        this.telemetry = telemetry;

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

    private void updateControls(Gamepad gamepad){

        // Shooting
        if(gamepad.a)
            Shoot();
        else
            StopMotors();

        if(gamepad.y)
            blockerUp();
        else
            blockerDown();


    }

    public void updateFlywheelControl(double targetTPS) {
        double currentTPS = flywheel.getVelocity();
        double ff = (kV * targetTPS) + (kS * Math.signum(targetTPS));
        double fb = kP * (targetTPS - currentTPS);

        double power = (ff + fb) * (12.0 / battery.getVoltage());
        flywheel.setPower(Math.max(-1.0, Math.min(1.0, power)));
    }

    public void Shoot() {
        LLResult result = cam.getLatestResult();
        if (result != null && result.isValid()) {
            updateVars(result);
        }

        updateFlywheelControl(shootVel);
        angleAdjuster.setPosition(anglePos);

        fol.update();
    }

    public double getDistance() {
        LLResult result = cam.getLatestResult();
        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult res : result.getFiducialResults()) {
                if (res.getFiducialId() == 20 || res.getFiducialId() == 24) {
                    double angle = 25.2 + res.getTargetYDegrees();
                    return (0.646 / Math.tan(Math.toRadians(angle))) + 0.2;
                }
            }
        }
        return 1.5;
    }

    private void updateVars(LLResult result) {
        for (LLResultTypes.FiducialResult res : result.getFiducialResults()) {
            if (res.getFiducialId() == 20 || res.getFiducialId() == 24) {
                double angle = 25.2 + res.getTargetYDegrees();
                double limeDist = (0.646 / Math.tan(Math.toRadians(angle))) + 0.2;

                beltSpeed = (limeDist < 2.6) ? 0.65 : 0.3;
                setShootPos(limeDist);
            }
        }
    }

    private void setShootPos(double dist) {
        double distMult = dist * 1.2;
        double veloMult = 2.21 + (distMult * 0.15);


        double targetAngle = Math.toDegrees(Math.atan(54.88 / (9.8 * distMult)));
        double rawVel = Math.sqrt((MAX_HEIGHT * 19.6) / Math.pow(Math.sin(Math.toRadians(targetAngle)), 2)) * veloMult;

        shootVel = (rawVel / (9.6 * Math.PI)) * 2800; // Vel to TPS
        interpolateAngle(distMult);
    }

    // LERP
    public void interpolateAngle(double distance) {
        Map.Entry<Double, Double> low = angleMap.floorEntry(distance);
        Map.Entry<Double, Double> high = angleMap.ceilingEntry(distance);

        if (low == null && high == null) return;
        if (low == null) anglePos = high.getValue();
        else if (high == null) anglePos = low.getValue();
        else {
            anglePos = low.getValue() + (distance - low.getKey()) * ((high.getValue() - low.getValue()) / (high.getKey() - low.getKey()));
        }
        anglePos = Math.max(0, Math.min(1, anglePos));
    }

    public void RunBelt(double speed) { belt.setPower(speed); }

    public void stopBelt() {belt.setPower(0); }

    public void StopMotors() {
        flywheel.setPower(0);
        stopBelt();
    }

    public void blockerUp(){
        feeder.setPosition(closePos);
    }

    public void blockerDown(){
        feeder.setPosition(openPos);
    }
    public void adjustServoManual(boolean up, boolean down) {
        double increment = 0.001;
        if (up) {manualServoPos += increment;}
        else if (down) { manualServoPos -= increment;}
        manualServoPos = Math.clamp(manualServoPos, 0, 1); angleAdjuster.setPosition(manualServoPos);

    }

    public Command runBeltCommand(double speed) {
        return new InstantCommand(() -> RunBelt(speed));
    }

    public Command stopBeltCommand() {
        return new InstantCommand(this::stopBelt);
    }

    public Command shootCommand(double beltPower, double durationMs) {
        ElapsedTime timer = new ElapsedTime();
        return new LambdaCommand("shootCommand")
                .setStart(timer::reset)
                .setUpdate(() -> {
                    this.Shoot();

                    if (timer.milliseconds() > 900) feeder.setPosition(closePos);
                    else feeder.setPosition(openPos);

                    // Belt Logic
                    if (timer.milliseconds() > 500 && (Math.abs(shootVel - flywheel.getVelocity()) < 50 || timer.milliseconds() > 700)) {
                        RunBelt(beltPower);
                    }
                })
                .setStop(interrupted -> {
                    StopMotors();
                    feeder.setPosition(openPos);
                })
                .setIsDone(() -> timer.milliseconds() > durationMs);
    }



}
