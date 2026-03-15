package org.firstinspires.ftc.teamcode.subsystems;

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
    public static final double downPos = .35, upPos = 0, extendPos = 1, retractPos = 0.1;
    public static final double IDLE_VELO = 700;

    // Sensor Vars
    private final VoltageSensor battery;
    public final Limelight3A cam;

    // Motor and Servo Vars
    public final Servo angleAdjuster, feeder, blocker;
    public final DcMotorEx belt, flywheel;

    // Servo Angle Stuff
    private final TreeMap<Double, Double> angleMap = new TreeMap<>();
    public double anglePos = 0.5, shootVel, beltSpeed = 1, manualServoPos = 0.15;
    private final double MAX_HEIGHT = 1.4;
    public double rawVelocity;
    public double tagDistance;

    public ControlSystem(HardwareMap hardwareMap, Telemetry telemetry) {
        belt = hardwareMap.get(DcMotorEx.class, "belt");
        flywheel = hardwareMap.get(DcMotorEx.class, "cannon");
        angleAdjuster = hardwareMap.get(Servo.class, "angleServo");
        feeder = hardwareMap.get(Servo.class, "feeder");
        blocker = hardwareMap.get(Servo.class, "blocker");
        battery = hardwareMap.voltageSensor.iterator().next();
        cam = hardwareMap.get(Limelight3A.class, "limelight");

        // Motor init
        flywheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel.setDirection(DcMotorEx.Direction.REVERSE);
        belt.setDirection(DcMotorEx.Direction.REVERSE);
        belt.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        blocker.setPosition(extendPos);
        cam.pipelineSwitch(0);
        cam.start();
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
                double angle = 25.2 + res.getTargetYDegrees();
                double limeDist = (0.646 / Math.tan(Math.toRadians(angle))) + 0.2;

                beltSpeed = (limeDist < 2.2) ? 0.8 : 0.45;
                setShootPos(limeDist);
            }
    }

    private void setShootPos(double dist) {
        double effectiveDist = dist * 1.2;
        double veloMult = 2.21 + (effectiveDist * 0.15);
        tagDistance = dist;

        double targetAngle = Math.toDegrees(Math.atan(54.88 / (9.8 * effectiveDist)));
        double rawVel = Math.sqrt((MAX_HEIGHT * 19.6) / Math.pow(Math.sin(Math.toRadians(targetAngle)), 2));
        rawVelocity = rawVel;

        anglePos = ShootSystem.calcHoodPos(effectiveDist);
        shootVel = ((rawVel * veloMult) / (9.6 * Math.PI)) * 2800; // Vel to TPS
    }

    public double ServoPosToRadians(double pos) { return Math.toRadians(90 - (pos * 300)); }

    public void RunBelt(double speed) { belt.setPower(speed); }

    public void SpitFlywheel() { flywheel.setPower(-0.05); }

    public void ExtendBlocker() { blocker.setPosition(extendPos); }

    public void RetractBlocker() { blocker.setPosition(retractPos); }

    public void stopBelt() { belt.setPower(0); }

    public void StopMotors() {
        flywheel.setPower(0);
        stopBelt();
    }
}