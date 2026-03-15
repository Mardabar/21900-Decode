package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.paths.CloseRed15Paths;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShootSystem;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;


@Disabled
@Autonomous(name = "Close Red 15")
public class CloseRed15 extends NextFTCOpMode{

    ShootSystem shootSystem;
    CloseRed15Paths paths;

    public CloseRed15(){
        addComponents(new PedroComponent(Constants::createFollower));
    }

    private SequentialGroup autonomousRoutine(){
        shootSystem = new ShootSystem(hardwareMap, telemetry);
        paths = new CloseRed15Paths(PedroComponent.follower());


        PedroComponent.follower().setStartingPose(paths.startPose);
        PedroComponent.follower().setMaxPower(1.0);

        return new SequentialGroup(

                shootSystem.spinUpCommand(),

                new FollowPath(paths.pathPreScore),

                shootSystem.shootClose(0.6, 1500),


                new ParallelGroup(
                        new FollowPath(paths.pathRow2Line),
                        shootSystem.blockerOut()
                ),


//                new FollowPath(paths.pathRow2Grab)
//                        .asDeadline(shootSystem.runBeltCommand(0.8)),
                new ParallelGroup(
                        new FollowPath(paths.pathRow2Grab),
                        shootSystem.runBeltCommand(1)
                ),




                new FollowPath(paths.pathOpenGate),
                shootSystem.stopBeltCommand(),

                new Delay(.4),

                shootSystem.spinUpCommand(),

                new ParallelGroup(
                        new FollowPath(paths.pathRow2Score),
                        shootSystem.blockerIn()
                ),
                shootSystem.shootClose(1, 1500),


                //new InstantCommand(() -> PedroComponent.follower().setMaxPower(.85)),
                new ParallelGroup(
                        new FollowPath(paths.pathFarmGate),
                        shootSystem.blockerOut()
                ),

//                new FollowPath(paths.pathGrabFromGate),


                shootSystem.runBeltForTime(1, 1.25),
                shootSystem.stopBeltCommand(),

                shootSystem.spinUpCommand(),


                new ParallelGroup(
                        new FollowPath(paths.pathRow2Score),
                        shootSystem.blockerIn()
                ),
                shootSystem.shootClose(0.8, 1500),




                new ParallelGroup(
                        new FollowPath(paths.pathFarmGate),
                        shootSystem.blockerOut()
                ),

//                new FollowPath(paths.pathGrabFromGate),


                shootSystem.runBeltForTime(1, 1.25),
                shootSystem.stopBeltCommand(),

                shootSystem.spinUpCommand(),


                new ParallelGroup(
                        new FollowPath(paths.pathRow2Score),
                        shootSystem.blockerIn()
                ),
                shootSystem.shootClose(0.8, 1500),




                new ParallelGroup(
                        new FollowPath(paths.pathRow1Line),

                        shootSystem.blockerOut()
                ),



                new FollowPath(paths.pathRow1Grab)
                        .asDeadline(shootSystem.runBeltCommand(0.8)),

                shootSystem.stopBeltCommand(),


                new ParallelGroup(
                        new FollowPath(paths.pathRow1Score),
                        shootSystem.blockerIn()
                ),

                shootSystem.shootClose(0.8, 1500),



//                new ParallelGroup(
//                        new FollowPath(paths.pathRow3Line),
//                        shootSystem.blockerOut()),



//                new FollowPath(paths.pathRow3Grab)
//                        .asDeadline(shootSystem.runBeltCommand(0.8)),
//
//
//                new InstantCommand(() -> PedroComponent.follower().setMaxPower(1.0)),
//                shootSystem.stopBeltCommand(),
//
//                new ParallelGroup(
//                        new FollowPath(paths.pathRow3Score),
//                        shootSystem.blockerIn()
//                ),
//                shootSystem.shootClose(0.8, 1600),

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
