package org.firstinspires.ftc.teamcode.config.subsystems;


import static org.firstinspires.ftc.teamcode.config.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.config.subsystems.ControlSystem.IDLE_VELO;


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
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.SequentialGroup;

import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;


import java.util.Map;
import java.util.TreeMap;



public class ShootSystem {

    public final Servo angleAdjuster, feeder, blocker;
    public final DcMotorEx belt, flywheel;

    private final TreeMap<Double, Double> angleMap = new TreeMap<>();
    public final VoltageSensor battery;
    public final Follower fol;
    public final Limelight3A cam;
    public final Telemetry telemetry;

    public double anglePos = 0.5, shootVel, beltSpeed = 1, manualServoPos = 0.15, manualBlockerPos = 0.1;
    public static double kP = 0.007, kS = 0.02, kV = 0.00045;
    public final double downPos = .35, upPos = 0, inPos = 0.1, outPos = 1;
    public static final double MAX_HEIGHT = 1.4;

    // Equation: y = ax^2 + bx + c
    //y=-0.0454854x^{2}+0.217006x-0.023787
    public static double hoodA = -0.0454854;
    public static double hoodB = 0.217006;
    public static double hoodC = -0.023787;


    public ShootSystem(HardwareMap hardwareMap, Telemetry telemetry){
        this.fol = follower;
        this.telemetry = telemetry;

        // Motors
        belt = hardwareMap.get(DcMotorEx.class, "belt");
        flywheel = hardwareMap.get(DcMotorEx.class, "cannon");

        // Servos
        angleAdjuster = hardwareMap.get(Servo.class, "angleServo");
        feeder = hardwareMap.get(Servo.class, "feeder");
        blocker = hardwareMap.get(Servo.class, "blocker");

        // Extra
        cam = hardwareMap.get(Limelight3A.class, "limelight");
        battery = hardwareMap.voltageSensor.iterator().next();

        // Init
        flywheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setDirection(DcMotorEx.Direction.REVERSE);
        belt.setDirection(DcMotorEx.Direction.REVERSE);
        belt.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        blocker.setDirection(Servo.Direction.REVERSE);
        //initDistances();
        blocker.setPosition(.1);
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

         /*New LERP data
        angleMap.put(0.489712, 0.129);
        angleMap.put(0.633476, 0.129);
        angleMap.put(0.7726934, 0.166);
        angleMap.put(0.8993826, 0.166);
        angleMap.put(1.016, 0.17);
        angleMap.put(1.1684, 0.18);
        angleMap.put(1.2790424, 0.194);
        angleMap.put(1.4224, 0.21);
        angleMap.put(2.9718, .23); */

    }

    private void calcHoodPos(double distance) {
        // Impl the quadratic formula y = ax^2 + bx + c
        anglePos = (hoodA * Math.pow(distance, 2)) + (hoodB * distance) + hoodC;

        anglePos = Math.max(0.12, Math.min(0.25, anglePos));
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

        if (Math.abs(shootVel - flywheel.getVelocity()) < 300) {
            blockIn();
        } else {
            blockOut();
        }

    }

    public void AutoShoot() {
        LLResult result = cam.getLatestResult();
        if (result != null && result.isValid()) {
            updateVars(result);
        }

        updateFlywheelControl(shootVel);
        angleAdjuster.setPosition(anglePos);



    }

    public void TestShoot() {
        LLResult result = cam.getLatestResult();
        if (result != null && result.isValid()) {
            updateVars(result);
        }
        updateFlywheelControl(shootVel);
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

                if (limeDist < .5)
                    beltSpeed = 1;
//                else if (limeDist < 1)
//                    beltSpeed = .8;
//                else if (limeDist < 1.5)
//                    beltSpeed = .7;
                else if(limeDist < 2.2)
                    beltSpeed = .8;
                else if (limeDist < 2.8)
                    beltSpeed = .45;

                setShootPos(limeDist);
            }
        }
    }

    private void oldSetShootPos(double dist) {
        double distMult = dist * 1.2;
        double veloMult = 2.21 + (distMult * 0.15);

        double targetAngle = Math.toDegrees(Math.atan(54.88 / (9.8 * distMult)));
        double rawVel = Math.sqrt((MAX_HEIGHT * 19.6) / Math.pow(Math.sin(Math.toRadians(targetAngle)), 2)) * veloMult;

        shootVel = (rawVel / (9.6 * Math.PI)) * 2800; // Vel to TPS
        interpolateAngle(distMult);
    }


    private void setShootPos(double dist) {
        double effectiveDist = dist * 1.2;
        double veloMult = 2.21 + (effectiveDist * 0.15);

        double targetAngle = Math.toDegrees(Math.atan(54.88 / (9.8 * effectiveDist)));
        double rawVel = Math.sqrt((MAX_HEIGHT * 19.6) / Math.pow(Math.sin(Math.toRadians(targetAngle)), 2)) * veloMult;
        shootVel = (rawVel / (9.6 * Math.PI)) * 2800;
        calcHoodPos(effectiveDist);
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
    
    public void StopWheel() {
        flywheel.setPower(0);
    }

    public void reverseWheel() {
        flywheel.setVelocity(-IDLE_VELO);
    }

    public void feederUp(){
        feeder.setPosition(upPos);
    }
    public void feederDown(){
        feeder.setPosition(downPos);
    }
    
    public void blockOut(){
        blocker.setPosition(outPos);
    }

    public void blockIn(){
        blocker.setPosition(inPos);
    }

    public void adjustServoManual(boolean up, boolean down) {
        double increment = 0.005;
        if (up) {manualServoPos += increment;}
        else if (down) { manualServoPos -= increment;}
        manualServoPos = Math.clamp(manualServoPos, 0, 1); angleAdjuster.setPosition(manualServoPos);
    }

    public void adjustBlockerManual(boolean up, boolean down) {
        double increment = 0.01;
        if (up) {manualBlockerPos += increment;}
        else if (down) { manualBlockerPos -= increment;}
        manualBlockerPos = Math.clamp(manualBlockerPos, 0, 1); blocker.setPosition(manualBlockerPos);
    }
    
    // Commands

    public Command runBeltCommand(double speed) {
        return new InstantCommand(() -> RunBelt(speed));
    }

    public Command stopBeltCommand() {
        return new InstantCommand(this::stopBelt);
    }

    public Command stopFlyWheel(){
        return new InstantCommand(this::StopWheel);
    }

    public Command reverseFlyWheel(){
        return new InstantCommand(this::reverseWheel);
    }

    public Command blockerIn(){
        return new InstantCommand(this::blockIn);
    }

    public Command blockerOut(){
        return new InstantCommand(this::blockOut);
    }
 


    public Command shootClose(double beltPower, double durationMs) {
        ElapsedTime timer = new ElapsedTime();
        return new LambdaCommand("shootCommand")
                .setStart(timer::reset)
                .setUpdate(() -> {
                    this.AutoShoot();

                    if (timer.milliseconds() > durationMs - 400)
                        feederUp();
                    else
                        feederDown();

                    if (timer.milliseconds() > 500 && (Math.abs(shootVel - flywheel.getVelocity()) < 50 || timer.milliseconds() > 700)) {
                        RunBelt(beltPower);
                    }
                })
                .setStop(interrupted -> {
                    StopMotors();
                    feederDown();
                })
                .setIsDone(() -> timer.milliseconds() > durationMs);
    }

    public Command shootFar(double beltPower, double durationMs) {
        ElapsedTime timer = new ElapsedTime();
        return new LambdaCommand("shootCommand")
                .setStart(timer::reset)
                .setUpdate(() -> {
                    this.AutoShoot();

                    if (timer.milliseconds() > durationMs - 550)
                        feederUp();
                    else
                        feederDown();

                    if (timer.milliseconds() > 500 && (Math.abs(shootVel - flywheel.getVelocity()) < 50 || timer.milliseconds() > 700)) {
                        RunBelt(beltPower);
                    }
                })
                .setStop(interrupted -> {
                    StopMotors();
                    feederDown();
                })
                .setIsDone(() -> timer.milliseconds() > durationMs);
    }

    public Command runBeltForTime(double speed, double durationSec) {
        return new SequentialGroup(
                runBeltCommand(speed),
                new Delay(durationSec),
                stopBeltCommand()
        );
    }


    public Command runBeltActive(double speed){
        return new LambdaCommand("runBeltActive")
                .setStart(() -> RunBelt(speed))
                .setIsDone(() -> false)
                .setStop(interrupted -> stopBelt());
    }
    public Command runBeltWithDelay(double speed, double durationSeconds) {
        return runBeltActive(speed).endAfter(durationSeconds);
    }




}