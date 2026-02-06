package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.config.paths.CloseBluePaths;
import org.firstinspires.ftc.teamcode.config.paths.CloseRedPaths;
import org.firstinspires.ftc.teamcode.config.paths.DuckPaths;
import org.firstinspires.ftc.teamcode.config.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.config.subsystems.Shooter;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;



@Autonomous(name = "Duck")
public class DUCK extends NextFTCOpMode {

    Shooter shooter;
    DuckPaths paths;

    public DUCK() {
        addComponents(new PedroComponent(Constants::createFollower));
    }


    private SequentialGroup autonomousRoutine() {
        shooter = new Shooter(hardwareMap, telemetry);
        paths = new DuckPaths(PedroComponent.follower());

        PedroComponent.follower().setStartingPose(paths.startPose);

        return new SequentialGroup(

                new InstantCommand(() -> PedroComponent.follower().setMaxPower(1)),
                //new FollowPath(paths.path1),
                //new FollowPath(paths.path2),
                new FollowPath(paths.path3),
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
