package org.firstinspires.ftc.teamcode.opmode.teleop;



import static org.firstinspires.ftc.teamcode.config.subsystems.ControlSystem.IDLE_VELO;
import static org.firstinspires.ftc.teamcode.config.subsystems.ControlSystem.closePos;
import static org.firstinspires.ftc.teamcode.config.subsystems.ControlSystem.openPos;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.config.subsystems.ControlSystem;
import org.firstinspires.ftc.teamcode.config.subsystems.MathUtilities;
import org.firstinspires.ftc.teamcode.config.subsystems.ShootSystem;

import java.util.Arrays;

@Configurable
@TeleOp(name = "Test Tele")
public class TestTele extends OpMode {

    ShootSystem shooter;


    private Follower fol;
    // 72, 72 is middle, 27, 131.8, Math.toRadians(143) is blue auto starting
    private final Pose startingPose = new Pose(72, 72, Math.toRadians(180));

    private final Pose blueGoalPose = new Pose(11, 36);
    private DcMotorEx lb, rb, lf, rf;

    private final double p = 0.03, d = 0.00011;
    private double lastError;
    private double iSum;
    private boolean isShooting;
    private boolean isDriving = true;

    @Override
    public void init() {
        shooter = new ShootSystem(hardwareMap, telemetry);

        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rb = hardwareMap.get(DcMotorEx.class, "rb");
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        rf = hardwareMap.get(DcMotorEx.class, "rf");

        fol = Constants.createFollower(hardwareMap);
        fol.setPose(startingPose);
        fol.startTeleOpDrive();

        shooter.beltSpeed = 0.8;
    }

    @Override
    public void loop() {
        fol.update();

        if (!isShooting) {
            fol.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        }

        if (Math.abs(gamepad1.left_stick_x + gamepad1.left_stick_y + gamepad1.right_stick_x) > 0.1 && isShooting) {
            isDriving = true;
            fol.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        } else if (isDriving || isShooting) {
            for (LLResultTypes.FiducialResult res : shooter.cam.getLatestResult().getFiducialResults()) {
                int id = res.getFiducialId();
                if ((id == 20 || id == 24) && !isDriving)
                    PIDAdjusting(res);
            }
            isDriving = false;
        }

        if (gamepad1.a) {
            shooter.TestShoot();
        } else {
            shooter.StopMotors();
        }

        if (gamepad1.dpad_left || gamepad2.dpad_left) {
            shooter.flywheel.setVelocity(-IDLE_VELO);
        }

        shooter.adjustServoManual(gamepad1.dpad_up, gamepad1.dpad_down);


        if (gamepad1.x)
            shooter.RunBelt(1);
        else if (gamepad1.b)
            shooter.RunBelt(-1);
        else
            shooter.RunBelt(0);

        if (gamepad1.y) shooter.feeder.setPosition(closePos);
        else shooter.feeder.setPosition(openPos);

        telemetry.addData("Target TPS", shooter.shootVel);
        telemetry.addData("Actual TPS", shooter.flywheel.getVelocity());
        telemetry.addData("Servo Pos", shooter.angleAdjuster.getPosition());
        telemetry.addData("X pose", fol.getPose().getX());
        telemetry.addData("Y pose", fol.getPose().getY());
        telemetry.addData("Distance from Pos", MathUtilities.calculateDistance(fol.getPose(), blueGoalPose));
        telemetry.update();

    }


    private void PIDAdjusting(LLResultTypes.FiducialResult res) {
        double error = res.getTargetXDegrees();
        iSum += error;
        double derError = lastError - error;

        double power = (error * p) + (derError * d);
        lb.setPower(power);
        rb.setPower(-power);
        lf.setPower(power);
        rf.setPower(-power);

        lastError = error;
    }
}