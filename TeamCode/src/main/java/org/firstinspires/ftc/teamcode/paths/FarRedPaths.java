package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.subsystems.PoseHolder;

public class FarRedPaths {


    public final Pose startPose = new Pose(56, 8, Math.toRadians(90)).mirror();
    final Pose preScorePose = new Pose(58, 17, Math.toRadians(111)).mirror();
    final Pose cornerBallLine = new Pose(14, 18, Math.toRadians(-160)).mirror(), cornerBallGrab = new Pose(10, 10, Math.toRadians(-173)).mirror(), cornerBallScore = new Pose(58, 17, Math.toRadians(111)).mirror();
    final Pose cornerBallGrabCP1 = new Pose(16.6, 14).mirror(), cornerBallGrabCP2 = new Pose(12.5, 10.2).mirror(); // was 10, 16.7
    final Pose row3Line = new Pose(52, 36, Math.toRadians(180)).mirror(), row3Grab = new Pose(12, 36, Math.toRadians(180)).mirror(), row3Score = new Pose(58, 17, Math.toRadians(111)).mirror(), row3ScoreCP = new Pose(47.5, 73).mirror();
    final Pose park = new Pose(35, 9, Math.toRadians(90)).mirror(); // was 51 21

    public PathChain pathPreScore, pathCornerBallLine, pathCornerBallGrab, pathCornerBallScore, pathRow3Line, pathRow3Grab, pathRow3Score, pathPark;

    public FarRedPaths(Follower fol){
        PoseHolder.GlobalStartPose = park;
        pathPreScore = fol.pathBuilder()
                .addPath(new BezierLine(startPose, preScorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), preScorePose.getHeading())
                .build();

        pathCornerBallLine = fol.pathBuilder()
                .addPath(new BezierLine(preScorePose, cornerBallLine))
                .setLinearHeadingInterpolation(preScorePose.getHeading(), cornerBallLine.getHeading())
                .build();

        pathCornerBallGrab = fol.pathBuilder()
                .addPath(new BezierLine(cornerBallLine, cornerBallGrab))
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
                .addPath(new BezierLine(startPose, park))
                .setLinearHeadingInterpolation(startPose.getHeading(), park.getHeading())
                .build();
    }

}
