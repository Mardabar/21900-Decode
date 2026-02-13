package org.firstinspires.ftc.teamcode.config.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

public class CloseBluePaths {


    /// BOT WIDTH 15.375
    /// BOT LENGTH 15.75
//    /// START POSE
//    public final Pose startPose = new Pose(27, 131.8, Math.toRadians(143));
//    public final Pose preScorePose = new Pose(50, 115, Math.toRadians(146));
//    public final Pose row1Line = new Pose(51, 84, Math.toRadians(180)), row1Grab = new Pose(17, 84, Math.toRadians(180)), row1Score = new Pose(39.5, 102, Math.toRadians(135));
//    public final Pose row2Line = new Pose(51, 60, Math.toRadians(180)), row2Grab = new Pose(14, 60, Math.toRadians(180)), row2ScoreCP = new Pose(53, 58), row2Score = new Pose(50, 93, Math.toRadians(135));
//    public final Pose row3Line = new Pose(51, 35, Math.toRadians(180)), row3Grab = new Pose(14, 35, Math.toRadians(180)), row3Score = new Pose(48, 107, Math.toRadians(138));
//
//    final Pose row3ParkClose = new Pose(45, 72, Math.toRadians(138));

    public final Pose startPose = new Pose(27, 131.8, Math.toRadians(143));
    final Pose preScorePose = new Pose(50, 115, Math.toRadians(146));
    final Pose row1Line = new Pose(51, 84, Math.toRadians(180)), row1Grab = new Pose(23, 84, Math.toRadians(180)), row1Score = new Pose(39.5, 102, Math.toRadians(135));
    final Pose row2Line = new Pose(51, 60, Math.toRadians(180)), row2Grab = new Pose(15, 60, Math.toRadians(180)), row2ScoreCP = new Pose(53, 58), row2Score = new Pose(50, 93, Math.toRadians(135));
    final Pose row3Line = new Pose(51, 35, Math.toRadians(180)), row3Grab = new Pose(15, 35, Math.toRadians(180)), row3Score = new Pose(48, 107, Math.toRadians(138));
    final Pose row3ParkClose = new Pose(45, 72, Math.toRadians(138));
    



    public PathChain pathPreScore, pathRow1Line, pathRow1Grab, pathRow1Score, pathRow2Line, pathRow2Grab, pathRow2Score, pathRow3Line, pathRow3Grab,  pathRow3Score, pathPark;







    public CloseBluePaths(Follower fol){


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
                .addPath(new BezierCurve(row2Grab, row2ScoreCP, row2Score))
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

    }


}


