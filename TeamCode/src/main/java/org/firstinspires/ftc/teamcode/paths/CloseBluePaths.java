package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

public class CloseBluePaths {

    private Follower fol;
    private static final Pose startPose = new Pose(28,131, Math.toRadians(144));
    private final Pose preScorePose = new Pose(60, 104, Math.toRadians(146)); // PRE-LOAD SCORING POSITION
    private final Pose lineRow1 = new Pose(44.5, 84, Math.toRadians(0)); // Position
    private final Pose lineRow1CP = new Pose(91,84); // Control Point
    private final Pose grabRow1 = new Pose(30, 84, Math.toRadians(0)); // Position
    private final Pose scoreRow1 = new Pose(61, 78, Math.toRadians(132)); // Scoring Position
    private final Pose lineRow2 = new Pose(47, 60, Math.toRadians(0)); // Position
    private final Pose row2LineCP = new Pose(85, 60); // Control Point
    private final Pose grabRow2 = new Pose(30, 59.5, Math.toRadians(0)); // Position
    private final Pose scoreRow2 = new Pose(61, 78, Math.toRadians(132)); // Scoring Position

    private final Pose parkPose = new Pose(50, 72, Math.toRadians(132)); // Parking Position

    private PathChain pathPreScore, pathRow1Line, pathGrabRow1, pathScoreRow1, pathRow2Line, pathGrabRow2, pathScoreRow2, pathParkPose;

    public CloseBluePaths(Follower follower){

        pathPreScore = fol.pathBuilder()
                .addPath(new BezierLine(startPose, preScorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), preScorePose.getHeading())
                .build();

        pathRow1Line = fol.pathBuilder()
                .addPath(new BezierLine(preScorePose, lineRow1))
                .setLinearHeadingInterpolation(preScorePose.getHeading(), lineRow1.getHeading())
                //.setTangentHeadingInterpolation()
                .build();

        pathGrabRow1 = fol.pathBuilder()
                .addPath(new BezierLine(lineRow1, grabRow1))
                .setConstantHeadingInterpolation(grabRow1.getHeading())
                .build();

        pathScoreRow1 = fol.pathBuilder()
                .addPath(new BezierLine(grabRow1, scoreRow1))
                .setLinearHeadingInterpolation(grabRow1.getHeading(), scoreRow1.getHeading())
                .build();

        pathRow2Line = fol.pathBuilder()
                .addPath(new BezierLine(scoreRow1, lineRow2))
                .setLinearHeadingInterpolation(scoreRow1.getHeading(), lineRow2.getHeading())
                .build();

        pathGrabRow2 = fol.pathBuilder()
                .addPath(new BezierLine(lineRow2, grabRow2))
                .setConstantHeadingInterpolation(grabRow2.getHeading())
                .build();

        pathScoreRow2 = fol.pathBuilder()
                .addPath(new BezierLine(grabRow2, scoreRow2))
                .setLinearHeadingInterpolation(grabRow2.getHeading(), scoreRow2.getHeading())
                .build();

        pathParkPose = fol.pathBuilder()
                .addPath(new BezierLine(scoreRow2, parkPose))
                .setConstantHeadingInterpolation(parkPose.getHeading())
                .build();
    }

}

