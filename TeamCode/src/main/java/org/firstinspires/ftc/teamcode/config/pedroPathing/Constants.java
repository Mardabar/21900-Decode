package org.firstinspires.ftc.teamcode.config.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.util.MathUtils;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.subsystems.LimeLightSubsystem;


        // Length is 15.5
        // Width 15.75
@Configurable
public class Constants {




    public static FollowerConstants followerConstants = new FollowerConstants()
        .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.2, 0.06922094058768, 0.0019898073264081775 ))
        .translationalPIDFCoefficients(new PIDFCoefficients(0.2, 0, 0.0062, 0.022))
        .headingPIDFCoefficients(new PIDFCoefficients(1,0,0.01,0.01))
        .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.07,0,0.0025,0.6,0.05))


            .centripetalScaling(0.0005)
            .mass(9.2)
            .forwardZeroPowerAcceleration(-47.521)
            .lateralZeroPowerAcceleration(-75.435);


    public static MecanumConstants driveConstants = new MecanumConstants()
            // Check issue with either wheels or wiring of motors because drive code is not working atm
            .maxPower(1)
            .xVelocity(66.554)
            .yVelocity(55.075)
            .useBrakeModeInTeleOp(true)

            .rightFrontMotorName("rf")
            .rightRearMotorName("rb")
            .leftRearMotorName("lb")
            .leftFrontMotorName("lf")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);


    public static PathConstraints pathConstraints = new PathConstraints(0.95, 100, 1, 1);


            public static Follower createFollower(HardwareMap hardwareMap) {
                PinpointLocalizer pinpointLocalizer = new PinpointLocalizer(hardwareMap, localizerConstants);

                LimeLightSubsystem.fusionLocalizer = new FusionLocalizer(
                        pinpointLocalizer,
                        new double[]{0.5, 0.5, 0.05},
                        new double[]{1.0, 1.0, 0.1},
                        new double[]{4.0, 4.0, 0.04},
                        100
                );

        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .setLocalizer(LimeLightSubsystem.fusionLocalizer)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }



    public static PinpointConstants localizerConstants = new PinpointConstants()


            .forwardPodY(5.375) // 4.3125
            .strafePodX(-4.3125) // 0.375  -3.700 -5.375
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            //If running into issue with pods later keep in mind the x is y and y is x, no clue why but otherwise values wouldnt work
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);


}
