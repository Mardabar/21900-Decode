package org.firstinspires.ftc.teamcode.opmode.teleop;

import static org.firstinspires.ftc.teamcode.config.subsystems.FeedBackShootSystem.IDLE_VELO;
import static org.firstinspires.ftc.teamcode.config.subsystems.FeedBackShootSystem.closePos;
import static org.firstinspires.ftc.teamcode.config.subsystems.FeedBackShootSystem.openPos;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.config.subsystems.FeedBackShootSystem;


///  BLEUAFEHATGSWF why do we have so many teleop modeeasdssdsd
///
@Configurable
@TeleOp(name = "Tele V2 Leif")
public class TeleV2Leif extends OpMode {
    FeedBackShootSystem shooter;

    private enum ShootState { IDLE, SPIN_UP, FIRING }
    private final ShootState currentShootState = ShootState.IDLE;
    private final ElapsedTime shootStateTimer = new ElapsedTime();

    private Follower fol;
    private final Pose startingPose = new Pose(72, 72, Math.toRadians(0));

    private DcMotorEx lb, rb, lf, rf;

    private final double p = 0.03, d = 0.00011;
    private double lastError;
    private double iSum;
    private boolean isShooting;

    @Override
    public void init() {
        shooter = new FeedBackShootSystem(hardwareMap, telemetry);

        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rb = hardwareMap.get(DcMotorEx.class, "rb");
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        rf = hardwareMap.get(DcMotorEx.class, "rf");

        fol = Constants.createFollower(hardwareMap);
        fol.setStartingPose(startingPose);
        fol.update();

        fol.startTeleOpDrive();

        shooter.beltSpeed = 0.8;
    }

    @Override
    public void loop() {
        fol.update();

        if (!isShooting) {
            fol.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        }

        if (Math.abs(gamepad1.left_stick_x + gamepad1.left_stick_y + gamepad1.right_stick_x) > 0.25 && isShooting) {
            fol.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        } else if (isShooting) {
            fol.setTeleOpDrive(0, 0, 0, true);
            for (LLResultTypes.FiducialResult res : shooter.cam.getLatestResult().getFiducialResults()) {
                int id = res.getFiducialId();
                if ((id == 20 || id == 24))
                    PIDAdjusting(res);
            }
        }

        if (gamepad2.a) {
            isShooting = true;
            shooter.Shoot();
        } else {
            isShooting = false;
            shooter.StopMotors();
            shooter.beltSpeed = 0.8;
            iSum = 0;
        }

        if(gamepad1.dpad_left || gamepad2.dpad_left)
            shooter.flywheel.setVelocity(-IDLE_VELO);





        //belt stuff
        if (gamepad2.x)
            shooter.RunBelt(shooter.beltSpeed);
        else if (gamepad2.b)
            shooter.RunBelt(-shooter.beltSpeed);
        else
            shooter.RunBelt(0);

        if (gamepad2.y)
            shooter.feeder.setPosition(closePos);
        else
            shooter.feeder.setPosition(openPos);


        // prints data
        telemetry.addData("TUNING - Servo Pos", "%.4f", shooter.manualServoPos);
        telemetry.addData("Current State", currentShootState);
        telemetry.addData("Target TPS", shooter.shootVel);
        telemetry.addData("Actual TPS", shooter.flywheel.getVelocity());
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