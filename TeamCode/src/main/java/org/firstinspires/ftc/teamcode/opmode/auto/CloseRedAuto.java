package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.config.paths.CloseRedPaths;
import org.firstinspires.ftc.teamcode.config.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.config.subsystems.ShootSystem;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;



@Autonomous(name = "Close Red")
public class CloseRedAuto extends NextFTCOpMode {

    ShootSystem shootSystem;
    CloseRedPaths paths;

    public CloseRedAuto() {
        addComponents(new PedroComponent(Constants::createFollower));
    }


    private SequentialGroup autonomousRoutine() {
        shootSystem = new ShootSystem(hardwareMap, telemetry);
        paths = new CloseRedPaths(PedroComponent.follower());

        PedroComponent.follower().setStartingPose(paths.startPose);

        return new SequentialGroup(

                new InstantCommand(() -> PedroComponent.follower().setMaxPower(0.65)),
                new FollowPath(paths.pathPreScore),
                shootSystem.shootCommand(0.8, 1500),

                new FollowPath(paths.pathRow1Line),



                new FollowPath(paths.pathRow1Grab)
                        .asDeadline(shootSystem.runBeltCommand(0.3)),


                shootSystem.stopBeltCommand(),
                new InstantCommand(() -> PedroComponent.follower().setMaxPower(1.0)),

                new FollowPath(paths.pathRow1Score),
                shootSystem.shootCommand(0.8, 1700),

                new FollowPath(paths.pathRow2Line),
                new InstantCommand(() -> PedroComponent.follower().setMaxPower(0.65)),

                new FollowPath(paths.pathRow2Grab)
                        .asDeadline(shootSystem.runBeltCommand(0.3)),

                new InstantCommand(() -> PedroComponent.follower().setMaxPower(1.0)),
                shootSystem.stopBeltCommand(),

                new FollowPath(paths.pathRow2Score),
                shootSystem.shootCommand(0.8, 1700),

                new FollowPath(paths.pathRow3Line),
                new InstantCommand(() -> PedroComponent.follower().setMaxPower(0.65)),

                new FollowPath(paths.pathRow3Grab)
                        .asDeadline(shootSystem.runBeltCommand(0.3)),

                new InstantCommand(() -> PedroComponent.follower().setMaxPower(1.0)),
                shootSystem.stopBeltCommand(),

                new FollowPath(paths.pathRow3Score),
                shootSystem.shootCommand(0.8, 1700),

                new FollowPath(paths.pathPark),
                new InstantCommand(shootSystem::StopMotors)
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
