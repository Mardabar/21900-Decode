package org.firstinspires.ftc.teamcode.config.paths;

import com.pedropathing.geometry.Pose;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;


public class CloseGateBluePaths {

    public final Pose startPose = new Pose(28, 131.5, Math.toRadians(143));

    final Pose preScorePose = new Pose(50, 115, Math.toRadians(146));
    final Pose row2Line = new Pose(50, 60, Math.toRadians(180)), row2Grab = new Pose(14, 60, Math.toRadians(180)), row2Score = new Pose(50, 93, Math.toRadians(135)), row2ScoreCP = new Pose(53, 58);
    final Pose row2OpenGate = new Pose(17.8, 70, Math.toRadians(150)), row2OpenGateCP = new Pose(32, 65);
    final Pose row1Line = new Pose(50, 84, Math.toRadians(180)), row1Grab = new Pose(23, 84, Math.toRadians(180)), row1Score = new Pose(40, 102, Math.toRadians(135));
    final Pose row3OpenGate = new Pose(16, 70, Math.toRadians(180));
    final Pose row3Line = new Pose(51, 35, Math.toRadians(180)), row3Grab = new Pose(15, 35, Math.toRadians(180)), row3Score = new Pose(48, 107, Math.toRadians(138));



    public PathChain pathPreScore, pathRow2Line, pathRow2Grab, pathRow2OpenGate, pathRow2Score, pathRow1Line, pathRow1Grab, pathRow1Score, pathRow3OpenGate, pathRow3Line, pathRow3Grab, pathRow3Score;

    public CloseGateBluePaths(Follower fol){

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

        pathRow2OpenGate = fol.pathBuilder()
                .addPath(new BezierCurve(row2Grab, row2OpenGateCP, row2OpenGate))
                .setLinearHeadingInterpolation(row2Grab.getHeading(), row2OpenGate.getHeading())
                .build();

        pathRow2Score = fol.pathBuilder()
                .addPath(new BezierCurve(row2OpenGate, row2ScoreCP, row2Score))
                .setLinearHeadingInterpolation(row2OpenGate.getHeading(), row2Score.getHeading())
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

    }
}
