package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.FeedBackShootSystem;


@Autonomous(name = "FarBlue")
public class FarBlueAuto extends OpMode {

    ///  Paths

    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));
    private final Pose preScorePose = new Pose(62, 119, Math.toRadians(159)); // for close shooting do 66, 120
    private final Pose row3Line = new Pose (48, 35.5, Math.toRadians(180));
    private final Pose row3Grab = new Pose (15, 35.5, Math.toRadians(180));
    private final Pose row3Score = new Pose(55.5, 121, Math.toRadians(159));
    private final Pose row3ScoreCP = new Pose(102, 70);



    // Pedro vars
    private PathChain pathPreScore, pathRow3Line, pathRow3Grab, pathRow3Score;
    private Follower fol;
    private int pathState; // Current path #

    // Subsystem
    private FeedBackShootSystem shooter;

    // Timeers
    private ElapsedTime shootTimer, beltTimer;


    @Override
    public void init(){


        shooter = new FeedBackShootSystem(hardwareMap, telemetry);

        fol = Constants.createFollower(hardwareMap);
        fol.setStartingPose(startPose);
        shootTimer = new ElapsedTime();
        beltTimer = new ElapsedTime();

        // Pedro paths init
        buildPaths();
        setPathState(0);

    }




    @Override
    public void loop(){

        fol.update();
        autonomousPathUpdate();

    }



    private void autonomousPathUpdate(){
        switch(pathState){


            case 0:
                if (!fol.isBusy()) {
                    fol.followPath(pathPreScore);
                    setPathState(1);
                } break;

            case 1:
                if (!fol.isBusy()){
                    shootTimer.reset();
                    setPathState(-1);
                } break;

            case -1:
                shoot(2); // change back to 2
                break;

            case 2:
                if (!fol.isBusy()){
                    fol.followPath(pathRow3Line);
                    setPathState(3);
                } break;

            case 3:
                if (!fol.isBusy()){
                    fol.setMaxPower(.45);
                    shooter.RunBelt(.4);
                    fol.followPath(pathRow3Grab);
                    beltTimer.reset();
                    setPathState(4); // back to 4
                } break;

            // Bot goes to scoring pos
            case 4:
                if(!fol.isBusy()){
                    fol.setMaxPower(1);
                    shooter.stopBelt();
                    //stopBelt();
                    fol.followPath(pathRow3Score);
                    setPathState(-4);
                } break;

            // Bot checks for it to stop moving
            case -4:
                if(!fol.isBusy()){
                    shootTimer.reset();
                    setPathState(5);
                } break;

            // Bot does shooting here, need to add timer to check when the bot can move again
            case 5:
                if(!fol.isBusy()){
                    shoot(6);
                } break;



        }


    }


    private void buildPaths(){
        pathPreScore = fol.pathBuilder()
                .addPath(new BezierLine(startPose, preScorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), preScorePose.getHeading())
                .build();

        pathRow3Line = fol.pathBuilder()
                .addPath(new BezierLine(preScorePose, row3Line))
                .setLinearHeadingInterpolation(preScorePose.getHeading(), row3Line.getHeading())
                .build();

        pathRow3Grab = fol.pathBuilder()
                .addPath(new BezierLine(row3Line, row3Grab))
                .setLinearHeadingInterpolation(row3Line.getHeading(), row3Grab.getHeading())
                .build();

        //              Row 3 close score and shoot
        pathRow3Score = fol.pathBuilder()
                .addPath(new BezierCurve(row3Grab, row3ScoreCP, row3Score))
                .setLinearHeadingInterpolation(row3Grab.getHeading(), row3Score.getHeading())
                .build();

    }


    private void shoot(int nextState) {
        // updates and sets motors to power
        shooter.Shoot();

        if (shootTimer.milliseconds() > 900) {
            shooter.feeder.setPosition(FeedBackShootSystem.closePos);
        } else {
            shooter.feeder.setPosition(FeedBackShootSystem.openPos);
        }

        // lets the flywheel spin up for a bit might need to make bigger
        if (shootTimer.milliseconds() < 500) {
            shooter.stopBelt();
        }

        // after that checks if the flywheel is at the velocity or if we have spun for over 3 seconds
        else if (Math.abs(shooter.shootVel - shooter.flywheel.getVelocity()) < 500 || shootTimer.milliseconds() > 700) {
            shooter.RunBelt(0.8);

        }

        // After 4 seconds stop everything and move to the next path state incase sum gets messed up
        if (shootTimer.milliseconds() > 1900) {
            shooter.StopMotors();
            shooter.feeder.setPosition(FeedBackShootSystem.openPos);
            setPathState(nextState);
        }

    }

    /// PEDRO FUNCTIONS
    private void setPathState(int num){
        pathState = num;
    }


}
