package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.config.paths.CloseBluePaths;
import org.firstinspires.ftc.teamcode.config.paths.CloseGateBluePaths;
import org.firstinspires.ftc.teamcode.config.paths.CloseRedPaths;
import org.firstinspires.ftc.teamcode.config.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.config.subsystems.ShootSystem;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;



@Autonomous(name = "Close Red")
public class CloseRed extends NextFTCOpMode {

    ShootSystem shootSystem;
    CloseRedPaths paths;

    public CloseRed() {
        addComponents(new PedroComponent(Constants::createFollower));
    }


    private SequentialGroup autonomousRoutine() {
        shootSystem = new ShootSystem(hardwareMap, telemetry);
        paths = new CloseRedPaths(PedroComponent.follower());


        PedroComponent.follower().setStartingPose(paths.startPose);
        PedroComponent.follower().setMaxPower(.85);

        return new SequentialGroup(

                new FollowPath(paths.pathPreScore),

                shootSystem.shootClose(0.6, 1600),


                new ParallelGroup(
                        new FollowPath(paths.pathRow2Line),

                        shootSystem.blockerOut()
                ),


                new FollowPath(paths.pathRow2Grab)
                        .asDeadline(shootSystem.runBeltCommand(0.8)),

                shootSystem.stopBeltCommand(),

                new FollowPath(paths.pathR2OpenGate),
                shootSystem.stopBeltCommand(),

                new Delay(.5),


                new ParallelGroup(
                        new FollowPath(paths.pathRow2Score),
                        shootSystem.blockerIn()
                ),

                shootSystem.shootClose(0.8, 1600),


                new ParallelGroup(
                        new FollowPath(paths.pathRow1Line),
                        shootSystem.blockerOut()
                ),


                new FollowPath(paths.pathRow1Grab)
                        .asDeadline(shootSystem.runBeltCommand(0.8)),

                shootSystem.stopBeltCommand(),
                new FollowPath(paths.pathR1OpenGate),

                new Delay(.5),



                new ParallelGroup(
                        new FollowPath(paths.pathRow1Score),
                        shootSystem.blockerIn()
                ),
                shootSystem.shootClose(1, 1600),

                new ParallelGroup(
                        new FollowPath(paths.pathRow3Line),
                        shootSystem.blockerOut()),

                new FollowPath(paths.pathRow3Grab)
                        .asDeadline(shootSystem.runBeltCommand(0.8)),

                new InstantCommand(() -> PedroComponent.follower().setMaxPower(1.0)),
                shootSystem.stopBeltCommand(),

                new ParallelGroup(
                        new FollowPath(paths.pathRow3Score),
                        shootSystem.blockerIn()
                ),
                shootSystem.shootClose(0.8, 1600),

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
