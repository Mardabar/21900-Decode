package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.paths.FarRedPaths;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShootSystem;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Disabled
@Autonomous(name = "Far Red ")
public class FarRed extends NextFTCOpMode{

    ShootSystem shootSystem;

    FarRedPaths paths;

    public FarRed(){
        addComponents(new PedroComponent(Constants::createFollower));
    }

    private SequentialGroup autonomousRoutine() {
        shootSystem = new ShootSystem(hardwareMap, telemetry);
        paths = new FarRedPaths(PedroComponent.follower());

        PedroComponent.follower().setMaxPower(0.8);
        PedroComponent.follower().setStartingPose(paths.startPose);

        return new SequentialGroup(

                new FollowPath(paths.pathPreScore),
                shootSystem.shootFar(0.45, 4000),

                new ParallelGroup(
                        new FollowPath(paths.pathCornerBallLine)
                                .asDeadline(shootSystem.runBeltCommand(1)),

                        shootSystem.blockerOut()
                ),

                new FollowPath(paths.pathCornerBallGrab)
                        .asDeadline(shootSystem.runBeltCommand(1)),


                new FollowPath(paths.pathCornerBallScore),


                shootSystem.stopBeltCommand(),
                new Delay(.5),
                shootSystem.blockerIn(),


                shootSystem.shootFar(.45, 4000),

                new ParallelGroup(
                        new FollowPath(paths.pathRow3Line),
                        shootSystem.blockerOut()
                ),

                new FollowPath(paths.pathRow3Grab)
                        .asDeadline(shootSystem.runBeltCommand(1)),


                shootSystem.stopBeltCommand(),
                new ParallelGroup(
                        shootSystem.blockerIn(),
                        new FollowPath(paths.pathRow3Score)),

                shootSystem.shootFar(.45, 4000),

                new FollowPath(paths.pathPark),

                new InstantCommand(shootSystem::StopMotors),



                new FollowPath(paths.pathRow3Line),


                new FollowPath(paths.pathRow3Grab)
                        .asDeadline(shootSystem.runBeltCommand(0.3)),

                shootSystem.stopBeltCommand(),

                new FollowPath(paths.pathPark)






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
