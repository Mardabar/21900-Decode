package org.firstinspires.ftc.teamcode.config.pedroPathing.pedroTesting.TankOpModes;



import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.pedroPathing.pedroTesting.TankConfig.TankConfig;

@TeleOp(name = "Tank Tele Test")
public class TankTeleTest extends LinearOpMode {

    public void runOpMode() {

        Follower follower = TankConfig.setup(hardwareMap);



        waitForStart();
        while (opModeIsActive()) {
            follower.setTeleOpDrive(-gamepad1.left_stick_y, 0, -gamepad1.right_stick_x, true);

            follower.update();

            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            //telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }
    }
}
