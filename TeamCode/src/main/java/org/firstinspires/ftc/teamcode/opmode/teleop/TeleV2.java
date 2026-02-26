package org.firstinspires.ftc.teamcode.opmode.teleop;

import static org.firstinspires.ftc.teamcode.config.subsystems.ControlSystem.IDLE_VELO;
import org.firstinspires.ftc.teamcode.config.subsystems.ShootSystem;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.config.pedroPathing.Constants;

@Configurable
@TeleOp(name = "Tele V2")
public class TeleV2 extends OpMode {
    private ShootSystem shooter;

    private Follower fol;
    private final Pose startingPose = new Pose(72, 72, Math.toRadians(0));

    private DcMotorEx lb, rb, lf, rf;

    private final double p = 0.03, d = 0.00011;
    private double lastError;
    private double odoToLimeError = 10;
    private static final Pose BLUE_GOAL = new Pose(0, 144);
    private static final Pose RED_GOAL = new Pose(144, 144);
    public static boolean isRed;

    private boolean isShooting;

    @Override
    public void init() {
        shooter = new ShootSystem(hardwareMap, telemetry);

        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rb = hardwareMap.get(DcMotorEx.class, "rb");
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        rf = hardwareMap.get(DcMotorEx.class, "rf");

        fol = Constants.createFollower(hardwareMap);
        fol.update();
        fol.startTeleOpDrive();
    }

    @Override
    public void loop() {
        fol.update();

        if (gamepad2.dpad_left)
            isRed = false;
        else if (gamepad2.dpad_right)
            isRed = true;

        if (Math.abs(gamepad1.left_stick_x + gamepad1.left_stick_y + gamepad1.right_stick_x) > 0.1)
            fol.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        else {
            fol.setTeleOpDrive(0, 0, 0, true);
            if (isShooting)
                AdjustAngle();
        }

        if (gamepad2.a) {
            isShooting = true;
            shooter.blockerIn();
            shooter.Shoot(false);
        } else if (!gamepad2.b) {
            isShooting = false;
            shooter.blockerOut();
            shooter.StopMotors();
            shooter.beltSpeed = 0.8;
        }

        if (gamepad1.dpad_down || gamepad2.dpad_down)
            shooter.flywheel.setVelocity(-IDLE_VELO);

        if (gamepad1.dpad_up || gamepad2.dpad_up)
            shooter.blockerIn();
        else if (!isShooting)
            shooter.blockerOut();

        if (gamepad2.x)
            shooter.RunBelt(shooter.beltSpeed);
        else if (gamepad2.b) {
            shooter.RunBelt(-shooter.beltSpeed);
            if (!isShooting)
                shooter.reverseWheel();
        }
        else
            shooter.RunBelt(0);

        if (gamepad2.y)
            shooter.feederUp();
        else
            shooter.feederDown();

        // prints data
        telemetry.addData("TUNING - Servo Pos", "%.4f", shooter.manualServoPos);
        telemetry.addData("Target TPS", shooter.shootVel);
        telemetry.addData("Actual TPS", shooter.flywheel.getVelocity());
        telemetry.addLine();
        telemetry.addData("Distance", shooter.tagDistance);
        telemetry.addLine();
        telemetry.addData("Servo Position", shooter.anglePos);
        telemetry.addData("True Angle", shooter.ServoPosToRadians(shooter.anglePos));
        telemetry.addLine();
        telemetry.addData("Velocity", shooter.rawVelocity);
        telemetry.addData("Flywheel TPS", shooter.shootVel);
        telemetry.update();
    }

    private void AdjustAngle(){
        for (LLResultTypes.FiducialResult res : shooter.cam.getLatestResult().getFiducialResults()) {
            int id = res.getFiducialId();
            if ((id == 20 || id == 24))
                PIDAdjusting(res);
        }
    }

    private void PIDAdjusting(LLResultTypes.FiducialResult res) {
        double error = GetError(res);
        double derError = lastError - error;

        double power = (error * p) + (derError * d);
        lb.setPower(power);
        rb.setPower(-power);
        lf.setPower(power);
        rf.setPower(-power);

        lastError = error;
    }

    private double GetError(LLResultTypes.FiducialResult res){
        Pose goal = BLUE_GOAL;
        if (isRed) goal = RED_GOAL;
        double headingError = Math.toDegrees(fol.getHeading() -
                Math.abs(Math.atan2(goal.getY() - fol.getPose().getY(), goal.getX() - fol.getPose().getX())));

        if (Math.abs(headingError) - Math.abs(res.getTargetXDegrees()) < odoToLimeError)
            return res.getTargetXDegrees();
        return headingError;
    }
}