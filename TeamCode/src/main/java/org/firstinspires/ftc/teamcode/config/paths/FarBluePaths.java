package org.firstinspires.ftc.teamcode.config.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class FarBluePaths {


    public final Pose startPose = new Pose(56, 8, Math.toRadians(90));
    final Pose preScorePose = new Pose(58, 17, Math.toRadians(120));
    final Pose cornerBallLine = new Pose(14, 17, Math.toRadians(190)), cornerBallGrab = new Pose(10, 10, Math.toRadians(199)), cornerBallScore = new Pose(57, 16, Math.toRadians(112));
    final Pose cornerBallGrabCP1 = new Pose(10, 16.7), cornerBallGrabCP2 = new Pose(12.5, 10.2);
    final Pose row3Line = new Pose(50, 35.5, Math.toRadians(180)), row3Grab = new Pose(12, 35.5, Math.toRadians(180)), row3Score = new Pose(53, 18, Math.toRadians(112));
    final Pose park = new Pose(56, 31, Math.toRadians(112));

    public PathChain pathPreScore, pathCornerBallLine, pathCornerBallGrab, pathCornerBallScore, pathRow3Line, pathRow3Grab, pathRow3Score, pathPark;

    public FarBluePaths(Follower fol){

        pathPreScore = fol.pathBuilder()
                .addPath(new BezierLine(startPose, preScorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), preScorePose.getHeading())
                .build();

        pathCornerBallLine = fol.pathBuilder()
                .addPath(new BezierLine(preScorePose, cornerBallLine))
                .setLinearHeadingInterpolation(preScorePose.getHeading(), cornerBallLine.getHeading())
                .build();

        pathCornerBallGrab = fol.pathBuilder()
                .addPath(new BezierCurve(cornerBallLine, cornerBallGrabCP1, cornerBallGrabCP2, cornerBallGrab))
                .setLinearHeadingInterpolation(cornerBallLine.getHeading(), cornerBallGrab.getHeading())
                .build();

        pathCornerBallScore = fol.pathBuilder()
                .addPath(new BezierLine(cornerBallGrab, cornerBallScore))
                .setLinearHeadingInterpolation(cornerBallGrab.getHeading(), cornerBallScore.getHeading())
                .build();

        pathRow3Line = fol.pathBuilder()
                .addPath(new BezierLine(cornerBallScore, row3Line))
                .setLinearHeadingInterpolation(cornerBallScore.getHeading(), row3Line.getHeading())
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
