package org.firstinspires.ftc.teamcode.config.pedroPathing.pedroTesting;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.FusionLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.config.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.config.subsystems.LimeLightSystem;

public class FusionLocalizerTest extends OpMode {

    private Follower fol;
    private LimeLightSystem limeLightSystem;

    @Override
    public void init(){

        fol = Constants.createFollower(hardwareMap);
        fol.setStartingPose(new Pose(0, 0, 0));

        FusionLocalizer fusion = (FusionLocalizer) fol.getPoseTracker().getLocalizer();

        limeLightSystem = new LimeLightSystem(hardwareMap, fusion);
        fol = Constants.createFollower(hardwareMap);
        fol.startTeleOpDrive();

    }

    @Override
    public void loop(){

        fol.update();
        limeLightSystem.update();

        fol.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);

    }
}
