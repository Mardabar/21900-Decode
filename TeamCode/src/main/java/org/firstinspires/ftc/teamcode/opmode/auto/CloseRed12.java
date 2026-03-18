package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.config.paths.CloseRed12Paths;
import org.firstinspires.ftc.teamcode.config.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.config.subsystems.ShootSystem;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;



@Disabled
@Autonomous(name = "Close Red12")
public class CloseRed12 extends NextFTCOpMode {

    ShootSystem shootSystem;
    CloseRed12Paths paths;

    public CloseRed12() {
        addComponents(new PedroComponent(Constants::createFollower));
    }


    private SequentialGroup autonomousRoutine() {
        shootSystem = new ShootSystem(hardwareMap, telemetry);
        paths = new CloseRed12Paths(PedroComponent.follower());


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
                        .asDeadline(shootSystem.runBeltCommand(1)),

                new ParallelGroup(
                        new FollowPath(paths.pathRow2Score),
                        (shootSystem.runBeltCommand(1))
                ),
                shootSystem.stopBeltCommand(),
                new Delay(.5),

                shootSystem.blockerIn(),

                shootSystem.shootClose(0.8, 1600),


                new ParallelGroup(
                        new FollowPath(paths.pathRow1Line),
                        shootSystem.blockerOut()
                ),


                new FollowPath(paths.pathRow1Grab)
                        .asDeadline(shootSystem.runBeltCommand(1)),

//                new FollowPath(paths.pathR1OpenGate),
//



                new ParallelGroup(
                        new FollowPath(paths.pathRow1Score),
                        (shootSystem.runBeltCommand(1))
                        ),
                shootSystem.stopBeltCommand(),
                        new Delay(.5),
                        shootSystem.blockerIn(),
                        new Delay(.25),
                        shootSystem.shootClose(.8, 1600),

                        new ParallelGroup(
                                new FollowPath(paths.pathRow3Line),
                                shootSystem.blockerOut()
                        ),


                                new FollowPath(paths.pathRow3Grab)
                                .asDeadline(shootSystem.runBeltCommand(1)),




                                new ParallelGroup(
                                        new FollowPath(paths.pathRow3Score),
                                        (shootSystem.runBeltCommand(1))
                                ),
                                shootSystem.stopBeltCommand(),
                                new Delay(.5),
                                shootSystem.blockerIn(),
                                new Delay(.25),
                                shootSystem.shootClose(.8, 1600),

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
