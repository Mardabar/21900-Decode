package org.firstinspires.ftc.teamcode.pedroPathing.pedroTesting.TankOpModes;



import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.paths.tankPaths.TankSquarePaths;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.ShootSystem;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "Tank Square Test")
public class TankSquareAuto extends NextFTCOpMode {
    TankSquarePaths paths;
    ShootSystem shooter;

    public TankSquareAuto() {
        addComponents(new PedroComponent(Constants::createFollower));
    }


        private SequentialGroup autonomousRoutine() {
            paths = new TankSquarePaths(PedroComponent.follower());
            shooter = new ShootSystem(hardwareMap, telemetry);


            PedroComponent.follower().setStartingPose(paths.startPose);
            PedroComponent.follower().setMaxPower(.7);

            return new SequentialGroup(

                    new FollowPath(paths.path1),

                    new Delay(.5),

                    new FollowPath(paths.path2),

                    new Delay(.5),

                    new FollowPath(paths.path3),

                    new Delay(.5),

                    new FollowPath(paths.path4)
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
