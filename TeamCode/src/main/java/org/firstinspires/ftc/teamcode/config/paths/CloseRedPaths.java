package org.firstinspires.ftc.teamcode.config.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.config.subsystems.PoseHolder;

public class CloseRedPaths {


    /// BOT WIDTH 15.375
    /// BOT LENGTH 15.75

    public final Pose startPose = new Pose(27, 131.5, Math.toRadians(139)).mirror();
    final Pose preScorePose = new Pose(52, 115, Math.toRadians(146)).mirror();
    final Pose row1Line = new Pose(52, 84, Math.toRadians(180)).mirror(), row1Grab = new Pose(20, 84, Math.toRadians(180)).mirror(), row1Score = new Pose(40, 102, Math.toRadians(135)).mirror(), row1ScoreCP = new Pose(47, 67).mirror();
    final Pose row2Line = new Pose(52, 60, Math.toRadians(180)).mirror(), row2Grab = new Pose(14.5, 60, Math.toRadians(180)).mirror(), row2ScoreCP = new Pose(60, 70).mirror(), row2Score = new Pose(50, 93, Math.toRadians(135)).mirror();
    final Pose openGate = new Pose(18, 66, Math.toRadians(180)).mirror(), openGateCP = new Pose(36, 65).mirror();
    final Pose r2OpenGate = new Pose(17, 66, Math.toRadians(180)).mirror(), r2OpenGateCP = new Pose(30, 62).mirror();
    final Pose r1OpenGate = new Pose(16, 78, Math.toRadians(180)).mirror(), r1OpenGateCP = new Pose(42, 78).mirror();

    final Pose row3Line = new Pose(52, 36, Math.toRadians(180)).mirror(), row3Grab = new Pose(12, 36, Math.toRadians(180)).mirror(), row3Score = new Pose(48, 107, Math.toRadians(138)).mirror(), row3ScoreCP = new Pose(47.5, 73).mirror();
    final Pose row3ParkClose = new Pose(45, 72, Math.toRadians(138)).mirror();


    public PathChain pathPreScore, pathRow1Line, pathRow1Grab, pathRow1Score, pathRow2Line, pathRow2Grab, pathOpenGate, pathR2OpenGate, pathR1OpenGate, pathRow2Score, pathRow3Line, pathRow3Grab,  pathRow3Score, pathPark;

    public CloseRedPaths(Follower fol){
        PoseHolder.GlobalStartPose = row3ParkClose;
        pathPreScore = fol.pathBuilder()
                .addPath(new BezierLine(startPose, preScorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), preScorePose.getHeading())
                .build();


        pathRow2Line = fol.pathBuilder()
                .addPath(new BezierLine(preScorePose, row2Line))
                .setLinearHeadingInterpolation(preScorePose.getHeading(), row2Line.getHeading())
                .build();

        pathRow2Grab = fol.pathBuilder()
                .addPath(new BezierLine(row2Line, row2Grab))
                .setLinearHeadingInterpolation(row2Line.getHeading(), row2Grab.getHeading())
                .build();

        pathR2OpenGate = fol.pathBuilder()
                .addPath(new BezierCurve(row2Grab, r2OpenGateCP, r2OpenGate))
                .setLinearHeadingInterpolation(row2Grab.getHeading(), r2OpenGate.getHeading())
                .build();

        pathOpenGate = fol.pathBuilder()
                .addPath(new BezierCurve(row2Grab, openGateCP, openGate))
                .setLinearHeadingInterpolation(row2Grab.getHeading(), openGate.getHeading())
                .build();

        pathRow2Score = fol.pathBuilder()
                .addPath(new BezierCurve(r2OpenGate, row2ScoreCP, row2Score)) //
                .setLinearHeadingInterpolation(r2OpenGate.getHeading(), row2Score.getHeading())
                .build();

        pathRow1Line = fol.pathBuilder()
                .addPath(new BezierLine(preScorePose, row1Line))
                .setLinearHeadingInterpolation(preScorePose.getHeading(), row1Line.getHeading())
                .build();

        pathRow1Grab = fol.pathBuilder()
                .addPath(new BezierLine(row1Line, row1Grab))
                .setLinearHeadingInterpolation(row1Line.getHeading(), row1Grab.getHeading())
                .build();

        pathR1OpenGate = fol.pathBuilder()
                .addPath(new BezierCurve(row1Grab, r1OpenGateCP, r1OpenGate))
                .setLinearHeadingInterpolation(row1Line.getHeading(), r1OpenGate.getHeading())
                .build();

        pathRow1Score = fol.pathBuilder()
                .addPath(new BezierCurve(row1Grab, row1ScoreCP, row1Score))
                .setLinearHeadingInterpolation(row1Grab.getHeading(), row1Score.getHeading())
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


