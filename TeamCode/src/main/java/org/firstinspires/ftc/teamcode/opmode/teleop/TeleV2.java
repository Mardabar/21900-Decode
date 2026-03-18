
package org.firstinspires.ftc.teamcode.opmode.teleop;

import static org.firstinspires.ftc.teamcode.config.subsystems.ControlSystem.IDLE_VELO;

import org.firstinspires.ftc.teamcode.config.subsystems.PoseHolder;
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
    private Pose startingPose;

    private DcMotorEx lb, rb, lf, rf;

    private final double p = 0.03, d = 0.00011;
    private double lastError;
    private double odoToLimeError = 5;
    private boolean odoOff;
    private static final Pose BLUE_GOAL = new Pose(0, 144);
    private static final Pose RED_GOAL = new Pose(144, 144);
    public static boolean isRed;

    private boolean isShooting;

    @Override
    public void init() {
        shooter = new ShootSystem(hardwareMap, telemetry);
        shooter.blockerOut();

        lb = hardwareMap.get(DcMotorEx.class, "lb");
        rb = hardwareMap.get(DcMotorEx.class, "rb");
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        rf = hardwareMap.get(DcMotorEx.class, "rf");

        fol = Constants.createFollower(hardwareMap);
        startingPose = PoseHolder.GlobalStartPose != null ? PoseHolder.GlobalStartPose
                : new Pose(45, 72, Math.toRadians(148));
        // 45, 72, 148 blue
        // 99, 72, 32 red
        fol.setStartingPose(startingPose);
        fol.update();
        fol.startTeleOpDrive();
    }

    @Override
    public void loop() {
        fol.update();

        if (startingPose.getX() > 72)
            isRed = true;
        else
            isRed = false;

        if (Math.abs(gamepad1.left_stick_x + gamepad1.left_stick_y + gamepad1.right_stick_x) > 0.1)
            fol.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
        else {
            fol.setTeleOpDrive(0, 0, 0, true);
            if (isShooting)
                PIDAdjusting();
        }

        if (gamepad2.a) {
            isShooting = true;
            shooter.Shoot(false);
            VibrateController(shooter.shotReady);
        } else if (!gamepad2.b) {
            isShooting = false;
            shooter.StopMotors();
            shooter.beltSpeed = 0.8;
            shooter.shotReady = false;
            gamepad2.stopRumble();
        }

        if (gamepad2.left_bumper)
            odoOff = true;

        if (gamepad1.dpad_down || gamepad2.dpad_down)
            shooter.flywheel.setVelocity(-IDLE_VELO);

        if (gamepad1.dpad_up || gamepad2.dpad_up)
            shooter.blockIn();
        else if (!isShooting)
            shooter.blockOut();

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
        telemetry.addData("Angle Error", lastError);
        telemetry.addLine();
        telemetry.addData("Servo Position", shooter.anglePos);
        telemetry.addData("True Angle", shooter.ServoPosToRadians(shooter.anglePos));
        telemetry.addLine();
        telemetry.addData("Velocity", shooter.rawVelocity);
        telemetry.addData("Flywheel TPS", shooter.shootVel);
        telemetry.addLine();
        telemetry.addData("Odometry Status", !odoOff);
        telemetry.update();
    }

    private void VibrateController(boolean ready){
        if (ready)
            gamepad2.rumble(500);
        else
            gamepad2.stopRumble();
    }

    private LLResultTypes.FiducialResult AdjustAngle(){
        for (LLResultTypes.FiducialResult res : shooter.cam.getLatestResult().getFiducialResults()) {
            int id = res.getFiducialId();
            if (id == 20 && !isRed)
                return res;
            else if (id == 24 && isRed)
                return res;
        }
        return null;
    }

    private void PIDAdjusting() {
        double error = GetError(AdjustAngle());
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
        double limeError = res != null ? res.getTargetXDegrees() : 0;
        double headingError = Math.toDegrees(Math.abs(Math.atan2(Math.abs(goal.getY() - fol.getPose().getY()),
                Math.abs(goal.getX() - fol.getPose().getX()))) - fol.getHeading());

        if (Math.abs(limeError + headingError) < odoToLimeError)
            return limeError;
        return !odoOff ? -headingError : limeError;
    }
}
