package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.config.paths.OldFarBluePaths;
import org.firstinspires.ftc.teamcode.config.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.config.subsystems.ShootSystem;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "FarBlue")
public class OldFarBlue extends NextFTCOpMode{

    ShootSystem shootSystem;

    OldFarBluePaths paths;

    public OldFarBlue(){
        addComponents(new PedroComponent(Constants::createFollower));
    }

    private SequentialGroup autonomousRoutine() {
        shootSystem = new ShootSystem(hardwareMap, telemetry);
        paths = new OldFarBluePaths(PedroComponent.follower());

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
