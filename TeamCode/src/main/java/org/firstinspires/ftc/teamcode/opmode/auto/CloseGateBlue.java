package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.config.paths.CloseGateBluePaths;
import org.firstinspires.ftc.teamcode.config.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.config.subsystems.ShootSystem;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;


@Autonomous(name = "Close Gate Blue")
public class CloseGateBlue extends NextFTCOpMode{
    
    ShootSystem shootSystem;
    CloseGateBluePaths paths;
    
    public CloseGateBlue(){
        addComponents(new PedroComponent(Constants::createFollower));
    }
    
    private SequentialGroup autonomousRoutine(){
        shootSystem = new ShootSystem(hardwareMap, telemetry);
        paths = new CloseGateBluePaths(PedroComponent.follower());

        PedroComponent.follower().setStartingPose(paths.startPose);
        
        
        return new SequentialGroup(

                new FollowPath(paths.pathPreScore),

                shootSystem.shootCommand(0.8, 1500),

                new FollowPath(paths.pathRow2Grab)
                        .asDeadline(shootSystem.runBeltCommand(0.3)),

                new InstantCommand(() -> PedroComponent.follower().setMaxPower(1.0)),
                shootSystem.stopBeltCommand(),

                new FollowPath(paths.pathRow2Score),
                shootSystem.shootCommand(0.8, 1700),

                new InstantCommand(() -> PedroComponent.follower().setMaxPower(.6)),
                new FollowPath(paths.pathOpenGate)
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
