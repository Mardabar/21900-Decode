//package org.firstinspires.ftc.teamcode.opmode.teleop;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.pedropathing.geometry.Pose;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
//import org.firstinspires.ftc.teamcode.config.pedroPathing.Constants;
//import org.firstinspires.ftc.teamcode.config.subsystems.ShootSystem;
//
//@TeleOp(name = "Tele Blue")
//public class TeleBlue extends OpMode {
//    ShootSystem shooter;
//
//    private final Pose startingPose = new Pose(15, 72, Math.toRadians(180));
//
//    private DcMotorEx lb, rb, lf, rf;
//    final double p = 0.03, d = 0.00011;
//    private double lastError;
//    private boolean isShooting;
//
//    @Override
//    public void init() {
//        shooter = new ShootSystem(hardwareMap, telemetry);
//        shooter.setTargetSide(false);
//
//        lb = hardwareMap.get(DcMotorEx.class, "lb");
//        rb = hardwareMap.get(DcMotorEx.class, "rb");
//        lf = hardwareMap.get(DcMotorEx.class, "lf");
//        rf = hardwareMap.get(DcMotorEx.class, "rf");
//
//        shooter.fol = Constants.createFollower(hardwareMap);
//        shooter.fol.setStartingPose(startingPose);
//        shooter.fol.startTeleOpDrive();
//    }
//
//    @Override
//    public void loop() {
//        shooter.fol.update();
//
//        if (!isShooting)
//            shooter.fol.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
//
//
//        if (gamepad2.a) {
//            isShooting = true;
//            shooter.Shoot();
//
//            for (LLResultTypes.FiducialResult res : shooter.cam.getLatestResult().getFiducialResults()) {
//                if (res.getFiducialId() == 24 && !isShooting) {
//                    PIDAdjusting(res);
//                }
//            }
//        } else {
//            isShooting = false;
//            shooter.StopMotors();
//        }
//
//        if (gamepad2.x) {
//            shooter.RunBelt(1);
//        } else if (gamepad2.b) {
//            shooter.RunBelt(-1);
//        } else {
//            shooter.stopBelt();
//        }
//
//        if (gamepad2.y)
//            shooter.blockerUp();
//        else
//            shooter.blockerDown();
//
//        telemetry.addData("Target TPS", shooter.shootVel);
//        telemetry.addData("Actual TPS", shooter.flywheel.getVelocity());
//        telemetry.update();
//    }
//
//    private void PIDAdjusting(LLResultTypes.FiducialResult res) {
//        double error = res.getTargetXDegrees();
//        double derError = lastError - error;
//        double power = (error * p) + (derError * d);
//        lb.setPower(power); rb.setPower(-power);
//        lf.setPower(power); rf.setPower(-power);
//        lastError = error;
//    }
//}