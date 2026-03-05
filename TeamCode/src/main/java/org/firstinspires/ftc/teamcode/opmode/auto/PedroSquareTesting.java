package org.firstinspires.ftc.teamcode.opmode.auto;

import com.pedropathing.ftc.FollowerBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.config.paths.CloseBlueTestingPaths;
import org.firstinspires.ftc.teamcode.config.paths.PedroTestingSquarePaths;
import org.firstinspires.ftc.teamcode.config.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.config.subsystems.ShootSystem;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;



@Autonomous(name = "Close Blue Testing")
public class PedroSquareTesting extends NextFTCOpMode {

    ShootSystem shootSystem;
    PedroTestingSquarePaths paths;

    public PedroSquareTesting() {
        addComponents(new PedroComponent(Constants::createFollower));
    }


    private SequentialGroup autonomousRoutine() {
        shootSystem = new ShootSystem(hardwareMap, telemetry);
        paths = new PedroTestingSquarePaths(PedroComponent.follower());


        PedroComponent.follower().setStartingPose(paths.startPose);
        PedroComponent.follower().setMaxPower(1);

        return new SequentialGroup(

                new FollowPath(paths.pathSquare1),

                //new Delay(.5),

                new FollowPath(paths.pathSquare2),

                //new Delay(.5),

                new FollowPath(paths.pathSquare3),

                //new Delay(.5),

                new FollowPath(paths.pathSquare4)




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
