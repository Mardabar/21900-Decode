package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.config.paths.FarBluePaths;
import org.firstinspires.ftc.teamcode.config.paths.OldFarBluePaths;
import org.firstinspires.ftc.teamcode.config.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.config.subsystems.ShootSystem;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "Far Blue")
public class FarBlue extends NextFTCOpMode{

    ShootSystem shootSystem;

    FarBluePaths paths;

    public FarBlue(){
        addComponents(new PedroComponent(Constants::createFollower));
    }

    private SequentialGroup autonomousRoutine() {
        shootSystem = new ShootSystem(hardwareMap, telemetry);
        paths = new FarBluePaths(PedroComponent.follower());

        PedroComponent.follower().setStartingPose(paths.startPose);

        return new SequentialGroup(
                new InstantCommand(() -> PedroComponent.follower().setMaxPower(0.8)),
                new FollowPath(paths.pathPreScore),
                shootSystem.shootFar(0.3, 2500),

                new FollowPath(paths.pathRow3Line),
                new InstantCommand(() -> PedroComponent.follower().setMaxPower(0.8)),

                new FollowPath(paths.pathRow3Grab)
                        .asDeadline(shootSystem.runBeltCommand(0.5)),
//                shootSystem.runBeltForTime(0.5, 600),


                new InstantCommand(() -> PedroComponent.follower().setMaxPower(1.0)),
                shootSystem.stopBeltCommand(),

                new FollowPath(paths.pathRow3Score),
                shootSystem.shootClose(0.8, 1600),

                new FollowPath(paths.pathCornerBallLine),


                new FollowPath(paths.pathCornerBallGrab)
                        .asDeadline(shootSystem.runBeltCommand(0.4))

//
//                new FollowPath(paths.pathRow3Line),
//
//
//                new FollowPath(paths.pathRow3Grab)
//                        .asDeadline(shootSystem.runBeltCommand(0.3)),
//
//                shootSystem.stopBeltCommand()





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
