package org.firstinspires.ftc.teamcode.config.subsystems;

import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.FusionLocalizer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class LimeLightSystem {
    private Limelight3A cam;
    private FusionLocalizer fusion;

    public LimeLightSystem(HardwareMap hardwareMap, FusionLocalizer fusion) {
        this.cam = hardwareMap.get(Limelight3A.class, "cam");
        this.fusion = fusion;
        cam.pipelineSwitch(0);
        cam.start();
    }

    public void update() {
        LLResult result = cam.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();


            Pose pedroPose = convertToPedroPose(botpose);

            long captureTime = System.nanoTime() - (long)(result.getStaleness() * 1_000_000L);

            if (fusion != null) {
                fusion.addMeasurement(pedroPose, captureTime);
            }
        }
    }

    private Pose convertToPedroPose(Pose3D botPose) {
        double x = botPose.getPosition().x * 39.37;
        double y = botPose.getPosition().y * 39.37;
        double heading = Math.toRadians(botPose.getOrientation().getYaw());

        // Apply the 90-degree offset gork mentioned if your heading is off
        return new Pose(x, y, heading);
    }
}
