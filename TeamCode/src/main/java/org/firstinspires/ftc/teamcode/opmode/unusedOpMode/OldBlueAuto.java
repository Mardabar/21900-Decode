package org.firstinspires.ftc.teamcode.opmode.unusedOpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.config.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.config.subsystems.ControlSystem;

@Disabled
@Autonomous(name = "FarBlue")
public class OldBlueAuto extends OpMode {

    private final Pose startPose = new Pose(27, 131.8, Math.toRadians(143));
    private final Pose preScorePose = new Pose(50, 115, Math.toRadians(146));
    private final Pose row1Line = new Pose(48, 84, Math.toRadians(180)), row1Grab = new Pose(17, 84, Math.toRadians(180)), row1Score = new Pose(39.5, 102, Math.toRadians(135));
    private final Pose row2Line = new Pose(50, 60, Math.toRadians(180)), row2Grab = new Pose(8, 60, Math.toRadians(180)), row2ScoreCP = new Pose(53, 58), row2Score = new Pose(50, 93, Math.toRadians(135));
    private final Pose row3Line = new Pose(50, 35.5, Math.toRadians(180)), row3Grab = new Pose(8, 35.5, Math.toRadians(180)), row3Score = new Pose(48, 107, Math.toRadians(138));
    private final Pose park = new Pose(45, 72, Math.toRadians(138));

    private Follower fol;
    private int pathState;
    private PathChain pathPreScore, pathRow3Line, pathRow3Grab, pathRow3Score, pathRow2Line, pathRow2Grab, pathRow2Score, pathRow1Line, pathRow1Grab, pathRow1Score, pathPark;

    private ControlSystem shooter;
    private final ElapsedTime shootTimer = new ElapsedTime(), beltTimer = new ElapsedTime();

    @Override
    public void init() {
        shooter = new ControlSystem(hardwareMap, telemetry);
        fol = Constants.createFollower(hardwareMap);
        fol.setStartingPose(startPose);
        buildPaths();
        pathState = 0;
    }

    @Override
    public void loop() {
        fol.update();
        autonomousPathUpdate();
    }

    private void autonomousPathUpdate() {
        switch(pathState) {
            case 0:
                if (!fol.isBusy()) {
                    fol.followPath(pathPreScore);
                    pathState = 1;
                } break;

            case 1:
                if (!fol.isBusy()){
                    shootTimer.reset();
                    pathState = 2;
                } break;

            case 2:
                Shoot(3, 0.35, 3000);
                break;

            case 3:
                if (!fol.isBusy()){
                    fol.followPath(pathRow3Line);
                    pathState = 4;
                } break;

            case 4:
                if (!fol.isBusy()){
                    fol.setMaxPower(0.6);
                    shooter.RunBelt(0.35);
                    fol.followPath(pathRow3Grab);
                    beltTimer.reset();
                    pathState = 5;
                } break;

            case 5:
                if (!fol.isBusy()){
                    fol.setMaxPower(1.0);
                    shooter.stopBelt();
                    fol.followPath(pathRow3Score);
                    pathState = 6;
                } break;

            case 6:
                if (!fol.isBusy()){
                    shootTimer.reset();
                    pathState = 7;
                } break;

            case 7:
                Shoot(8, 0.35, 3000);
                break;

            case 8:
                if (!fol.isBusy()){
                    fol.followPath(pathRow2Line);
                    pathState = 9;
                } break;

            case 9:
                if (!fol.isBusy()){
                    fol.setMaxPower(0.6);
                    shooter.RunBelt(0.35);
                    fol.followPath(pathRow2Grab);
                    pathState = 10;
                } break;

            case 10:
                if (!fol.isBusy()){
                    fol.setMaxPower(1.0);
                    shooter.stopBelt();
                    fol.followPath(pathRow2Score);
                    pathState = 11;
                } break;

            case 11:
                if (!fol.isBusy()){
                    shootTimer.reset();
                    pathState = 12;
                } break;

            case 12:
                Shoot(13, 0.35, 3000);
                break;

            case 13:
                if (!fol.isBusy()){
                    fol.followPath(pathPark);
                    pathState = 14;
                } break;

            case 14:
                break;
        }
    }

    private void Shoot(int nextState, double beltPower, double duration) {
        shooter.Shoot();

        if (shootTimer.milliseconds() > 900)
            shooter.feeder.setPosition(ControlSystem.closePos);
        else
            shooter.feeder.setPosition(ControlSystem.openPos);

        if (shootTimer.milliseconds() > 500 && (Math.abs(shooter.shootVel - shooter.flywheel.getVelocity()) < 50 || shootTimer.milliseconds() > 700))
            shooter.RunBelt(beltPower);

        if (shootTimer.milliseconds() > duration) {
            shooter.StopMotors();
            shooter.feeder.setPosition(ControlSystem.openPos);
            pathState = nextState;
        }
    }

    private void buildPaths() {

        pathPreScore = fol.pathBuilder()
                .addPath(new BezierLine(startPose, preScorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), preScorePose.getHeading())
                .build();

        pathRow1Line = fol.pathBuilder()
                .addPath(new BezierLine(preScorePose, row1Line))
                .setLinearHeadingInterpolation(preScorePose.getHeading(), row1Line.getHeading())
                .build();

        pathRow1Grab = fol.pathBuilder()
                .addPath(new BezierLine(row1Line, row1Grab))
                .setLinearHeadingInterpolation(row1Line.getHeading(), row1Grab.getHeading())
                .build();

        pathRow1Score = fol.pathBuilder()
                .addPath(new BezierLine(row1Grab, row1Score))
                .setLinearHeadingInterpolation(row1Grab.getHeading(), row1Score.getHeading())
                .build();

        pathRow2Line = fol.pathBuilder()
                .addPath(new BezierLine(row1Score, row2Line))
                .setLinearHeadingInterpolation(row1Score.getHeading(), row2Line.getHeading())
                .build();

        pathRow2Grab = fol.pathBuilder()
                .addPath(new BezierLine(row2Line, row2Grab))
                .setLinearHeadingInterpolation(row2Line.getHeading(), row2Grab.getHeading())
                .build();

        pathRow2Score = fol.pathBuilder()
                .addPath(new BezierCurve(row2Grab, row2ScoreCP, row2Score)) .setLinearHeadingInterpolation(row2Grab.getHeading(), row2Score.getHeading())
                .build();

        pathRow3Line = fol.pathBuilder()
                .addPath(new BezierLine(row2Score, row3Line))
                .setLinearHeadingInterpolation(row2Score.getHeading(), row3Line.getHeading())
                .build();

        pathRow3Grab = fol.pathBuilder()
                .addPath(new BezierLine(row3Line, row3Grab))
                .setLinearHeadingInterpolation(row3Line.getHeading(), row3Grab.getHeading())
                .build();

        pathRow3Score = fol.pathBuilder()
                .addPath(new BezierLine(row3Grab, row3Score))
                .setLinearHeadingInterpolation(row3Grab.getHeading(), row3Score.getHeading())
                .build();

        pathPark = fol.pathBuilder()
                .addPath(new BezierLine(row3Score, park))
                .setLinearHeadingInterpolation(row3Score.getHeading(), park.getHeading())
                .build();
    }
}