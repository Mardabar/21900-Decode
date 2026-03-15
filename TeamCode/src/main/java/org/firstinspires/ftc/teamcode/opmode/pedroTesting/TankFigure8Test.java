package org.firstinspires.ftc.teamcode.opmode.pedroTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.paths.tankPaths.TankFigure8Paths;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShootSystem;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "Tank Figure 8 Test")
public class TankFigure8Test extends NextFTCOpMode {
    TankFigure8Paths paths;
    ShootSystem shooter;

    public TankFigure8Test() {
        addComponents(new PedroComponent(Constants::createFollower));
    }


    private SequentialGroup autonomousRoutine() {
        paths = new TankFigure8Paths(PedroComponent.follower());
        shooter = new ShootSystem(hardwareMap, telemetry);


        PedroComponent.follower().setStartingPose(paths.startPose);
        PedroComponent.follower().setMaxPower(.7);

        return new SequentialGroup(

                new FollowPath(paths.path1),


                new Delay(.5),

                new FollowPath(paths.path2),

                new Delay(.5),

                new FollowPath(paths.path3)

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
