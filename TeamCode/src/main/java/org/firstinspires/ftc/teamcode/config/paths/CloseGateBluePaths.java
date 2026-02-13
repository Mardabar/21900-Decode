package org.firstinspires.ftc.teamcode.config.paths;

import com.pedropathing.geometry.Pose;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;


public class CloseGateBluePaths {

    public final Pose startPose = new Pose(27, 131.8, Math.toRadians(143));

    final Pose preScorePose = new Pose(50, 115, Math.toRadians(146));
    final Pose row2Line = new Pose(51, 60, Math.toRadians(180)), row2Grab = new Pose(15, 60, Math.toRadians(180)), row2Score = new Pose(50, 93, Math.toRadians(135)), row2ScoreCP = new Pose(53, 58);
    final Pose openGate = new Pose(13, 60, Math.toRadians(135)), openGateCP = new Pose(53, 58);

    public PathChain pathPreScore, pathRow2Line, pathRow2Grab, pathRow2Score, pathOpenGate;

    public CloseGateBluePaths(Follower fol){

        pathPreScore = fol.pathBuilder()
                .addPath(new BezierLine(startPose, preScorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), preScorePose.getHeading())
                .build();

        pathRow2Line = fol.pathBuilder()
                .addPath(new BezierLine(preScorePose, row2Line))
                .setLinearHeadingInterpolation(preScorePose.getHeading(), row2Line.getHeading())
                .build();

        pathRow2Score = fol.pathBuilder()
                .addPath(new BezierCurve(row2Line, row2ScoreCP, row2Score))
                .setLinearHeadingInterpolation(row2Grab.getHeading(), row2Score.getHeading())
                .build();
    }
}
