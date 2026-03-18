package org.firstinspires.ftc.teamcode.pedroPathing.pedroTesting.TankConfig;


import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;




import com.pedropathing.ftc.localization.constants.PinpointConstants;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class TankConfig {
    public static Follower setup(HardwareMap hardwareMap) {

        TankConstants tConstants = new TankConstants();

        tConstants.setLeftFrontMotorName("lf");
        tConstants.setLeftRearMotorName("lb");
        tConstants.setRightFrontMotorName("rf");
        tConstants.setRightRearMotorName("rb");


        Drivetrain tankDrive = new Tank(hardwareMap, tConstants);



        return new FollowerBuilder(new FollowerConstants(), hardwareMap)
                .setDrivetrain(tankDrive)
                .pinpointLocalizer(new PinpointConstants())
                .build();
    }


}