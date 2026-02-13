package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.config.paths.FarBluePaths;
import org.firstinspires.ftc.teamcode.config.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.config.subsystems.ShootSystem;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "FarBlue")
public class Far9Blue extends NextFTCOpMode{

    ShootSystem shootSystem;

    FarBluePaths paths;

    public Far9Blue(){
        addComponents(new PedroComponent(Constants::createFollower));
    }

    private SequentialGroup autonomousRoutine() {
        shootSystem = new ShootSystem(hardwareMap, telemetry);
        paths = new FarBluePaths(PedroComponent.follower());

        PedroComponent.follower().setStartingPose(paths.startPose);

        return new SequentialGroup(
                new InstantCommand(() -> PedroComponent.follower().setMaxPower(0.65)),
                new FollowPath(paths.pathPreScore),
                shootSystem.shootCommand(0.8, 1500),

                new FollowPath(paths.pathRow3Line),


                new FollowPath(paths.pathRow3Grab)
                        .asDeadline(shootSystem.runBeltCommand(0.3)),

                shootSystem.stopBeltCommand()





        );


    }


    @Override
    public void onStartButtonPressed(){
        autonomousRoutine().schedule();
    }

    @Override
    public void onUpdate(){
        CommandManager.INSTANCE.run();

    }

}
