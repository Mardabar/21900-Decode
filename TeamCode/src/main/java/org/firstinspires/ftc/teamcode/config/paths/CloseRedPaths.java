package org.firstinspires.ftc.teamcode.config.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

public class CloseRedPaths {


    /// BOT WIDTH 15.375
    /// BOT LENGTH 15.75

    /// START POSE
    final Pose startPose = new Pose(27, 131.8, Math.toRadians(143)).mirror();
    final Pose preScorePose = new Pose(50, 115, Math.toRadians(146)).mirror();
    final Pose row1Line = new Pose(48, 84, Math.toRadians(180)).mirror(), row1Grab = new Pose(17, 84, Math.toRadians(180)).mirror(), row1Score = new Pose(39.5, 102, Math.toRadians(135)).mirror();
    final Pose row2Line = new Pose(50, 60, Math.toRadians(180)).mirror(), row2Grab = new Pose(8, 60, Math.toRadians(180)).mirror(), row2ScoreCP = new Pose(53, 58).mirror(), row2Score = new Pose(50, 93, Math.toRadians(135)).mirror();
    final Pose row3Line = new Pose(50, 35.5, Math.toRadians(180)).mirror(), row3Grab = new Pose(8, 35.5, Math.toRadians(180)).mirror(), row3Score = new Pose(48, 107, Math.toRadians(138)).mirror();
    final Pose row3ParkClose = new Pose(45, 72, Math.toRadians(138)).mirror();


    public PathChain pathPreScore, pathRow1Line, pathRow1Grab, pathRow1Score, pathRow2Line, pathRow2Grab, pathRow2Score, pathRow3Line, pathRow3Grab,  pathRow3Score, pathPark;







    public CloseRedPaths(Follower fol){

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
                .addPath(new BezierLine(row2Grab, row2Score))
                .setLinearHeadingInterpolation(row2Grab.getHeading(), row2Score.getHeading())
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
                .addPath(new BezierLine(row3Score, row3ParkClose))
                .setLinearHeadingInterpolation(row3Score.getHeading(), row3ParkClose.getHeading())
                .build();

        /*              Row 3 far score and shoot
        pathRow3Score = fol.pathBuilder()
                .addPath(new BezierLine(row3Grab, row3ScoreFar))
                .setLinearHeadingInterpolation(row3Grab.getHeading(), row3ScoreFar.getHeading())
                .build();

        pathPark = fol.pathBuilder()
                .addPath(new BezierLine(row3ScoreFar, row3ParkFar))
                .setLinearHeadingInterpolation(row3ScoreFar.getHeading(), row3ParkFar.getHeading())
                .build();  */


    }


}
