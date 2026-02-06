package org.firstinspires.ftc.teamcode.config.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

public class DuckPaths {

    public final Pose startPose = new Pose(56, 8, Math.toRadians(90));
    final Pose lineToBalls = new Pose(23, 12, Math.toRadians(20));
    final Pose runIntoBalls1 = new Pose(22, 105, Math.toRadians(140)), runintoballsCP = new Pose(4, 49), runIntoBallsCP2 = new Pose (40, 81);
    final Pose finalBalls = new Pose(128, 30, Math.toRadians(67));
    final Pose pigBenis = new Pose(120, 105, Math.toRadians(53));
    final Pose pigBenisCP = new Pose(94, 12);

    public PathChain path1, path2, path3, path4, path5;


    public DuckPaths(Follower follower){

        path1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, lineToBalls))
                .setLinearHeadingInterpolation(startPose.getHeading(), lineToBalls.getHeading())
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(lineToBalls, runIntoBalls1))
                .setLinearHeadingInterpolation(lineToBalls.getHeading(), runIntoBalls1.getHeading())
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierCurve(runIntoBalls1, runintoballsCP, finalBalls))
                .setLinearHeadingInterpolation(runIntoBalls1.getHeading(), finalBalls.getHeading())
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(finalBalls, pigBenisCP, pigBenis))
                .setLinearHeadingInterpolation(finalBalls.getHeading(), pigBenis.getHeading())
                .build();
    }

}
