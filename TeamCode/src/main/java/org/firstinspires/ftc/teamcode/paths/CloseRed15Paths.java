package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.geometry.Pose;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.subsystems.PoseHolder;


public class CloseRed15Paths {


    public final Pose startPose = new Pose(27, 131.5, Math.toRadians(139)).mirror();
    final Pose preScorePose = new Pose(52, 115, Math.toRadians(146)).mirror();
    final Pose row1Line = new Pose(52, 84, Math.toRadians(180)).mirror(), row1Grab = new Pose(20, 84, Math.toRadians(180)).mirror(), row1Score = new Pose(40, 102, Math.toRadians(135)).mirror(), row1ScoreCP = new Pose(47, 67).mirror();
    final Pose row2Line = new Pose(52, 60, Math.toRadians(180)).mirror(), row2Grab = new Pose(14.5, 60, Math.toRadians(180)).mirror(), row2ScoreCP = new Pose(60, 70).mirror(), row2Score = new Pose(50, 93, Math.toRadians(135)).mirror();
    final Pose openGate = new Pose(13.5, 60, Math.toRadians(180)).mirror(), openGateCP = new Pose(36, 65).mirror();

    final Pose row3Line = new Pose(52, 36, Math.toRadians(180)).mirror(), row3Grab = new Pose(12, 36, Math.toRadians(180)).mirror(), row3Score = new Pose(48, 107, Math.toRadians(138)).mirror(), row3ScoreCP = new Pose(47.5, 73).mirror();
    final Pose row3ParkClose = new Pose(45, 72, Math.toRadians(138)).mirror();

    final Pose farmGate = new Pose(15, 62, Math.toRadians(155)).mirror(), farmGateCP = new Pose(29, 60);
    final Pose farmGateCP2 = new Pose(66, 55).mirror();
    final Pose slapOpenGate = new Pose(12.5, 62, Math.toRadians(155)).mirror(), slapOpenGateCP = new Pose(30, 60).mirror();
    final Pose grabFromGate = new Pose(12, 53, Math.toRadians(144)).mirror(), grabFromGateCP = new Pose(17, 56).mirror();





    public PathChain pathPreScore, pathRow2Line, pathRow2Grab, pathRow2Score, pathRow1Line, pathRow1Grab, pathRow1Score, pathFarmGate, pathFarmGate2, pathOpenGate, pathSlapOpenGate, pathGrabFromGate, pathRow3Line, pathRow3Grab, pathRow3Score, pathPark;

    public CloseRed15Paths(Follower fol){
        PoseHolder.GlobalStartPose = row3ParkClose;
        pathPreScore = fol.pathBuilder()
                .addPath(new BezierLine(startPose, preScorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), preScorePose.getHeading())
                .build();

        pathRow1Line = fol.pathBuilder()
                .addPath(new BezierLine(row2Score, row1Line))
                .setLinearHeadingInterpolation(row2Score.getHeading(), row1Line.getHeading())
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
                .addPath(new BezierLine(preScorePose, row2Line))
                .setLinearHeadingInterpolation(preScorePose.getHeading(), row2Line.getHeading())
                .build();

        pathRow2Grab = fol.pathBuilder()
                .addPath(new BezierLine(row2Line, row2Grab))
                .setLinearHeadingInterpolation(row2Line.getHeading(), row2Grab.getHeading())
                .build();

        pathFarmGate = fol.pathBuilder()
                .addPath(new BezierCurve(row2Score, farmGate, farmGate))
                .setLinearHeadingInterpolation(row2Score.getHeading(), farmGate.getHeading())
                .build();

        pathFarmGate2 = fol.pathBuilder()
                .addPath(new BezierCurve(row2Score, farmGateCP2, farmGate))
                .setLinearHeadingInterpolation(row2Score.getHeading(), farmGate.getHeading())
                .build();

        pathOpenGate = fol.pathBuilder()
                .addPath(new BezierCurve(row2Grab, openGateCP, openGate))
                .setLinearHeadingInterpolation(row2Grab.getHeading(), openGate.getHeading())
                .build();

        pathRow2Score = fol.pathBuilder()
                .addPath(new BezierCurve(farmGate, row2ScoreCP, row2Score))
                .setLinearHeadingInterpolation(farmGate.getHeading(), row2Score.getHeading())
                .build();


        pathSlapOpenGate = fol.pathBuilder()
                .addPath(new BezierCurve(row2Score, slapOpenGateCP, slapOpenGate))
                .setLinearHeadingInterpolation(row2Score.getHeading(), slapOpenGate.getHeading())
                .build();

        pathGrabFromGate = fol.pathBuilder()
                .addPath(new BezierCurve(slapOpenGate, grabFromGateCP, grabFromGate))
                .setLinearHeadingInterpolation(slapOpenGate.getHeading(), grabFromGate.getHeading())
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
                .addPath(new BezierCurve(row3Grab,row3ScoreCP, row3Score))
                .setLinearHeadingInterpolation(row3Grab.getHeading(), row3Score.getHeading())
                .build();

        pathPark = fol.pathBuilder()
                .addPath(new BezierLine(row3Score, row3ParkClose))
                .setLinearHeadingInterpolation(row3Score.getHeading(), row3ParkClose.getHeading())
                .build();


    }
}
