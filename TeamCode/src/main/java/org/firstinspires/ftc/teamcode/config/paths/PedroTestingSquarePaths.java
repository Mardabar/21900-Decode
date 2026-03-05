package org.firstinspires.ftc.teamcode.config.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import dev.nextftc.extensions.pedro.FollowPath;

public class PedroTestingSquarePaths {

    public final Pose startPose = new Pose(56, 47, Math.toRadians(90));

    public final Pose squarePose1 = new Pose(100, 47, Math.toRadians(90)), squarePose2 = new Pose(100, 90, Math.toRadians(90));
    public final Pose squarePose3 = new Pose(56, 93, Math.toRadians(90)), squarePose4 = new Pose(56, 47, Math.toRadians(90));


    public PathChain pathSquare1, pathSquare2, pathSquare3, pathSquare4;

    public PedroTestingSquarePaths(Follower fol){

        pathSquare1 = fol.pathBuilder()
                .addPath(new BezierLine(startPose, squarePose1))
                .setVelocityConstraint(.1)
                .setLinearHeadingInterpolation(startPose.getHeading(), squarePose1.getHeading())
                .build();

        pathSquare2 = fol.pathBuilder()
                .addPath(new BezierLine(squarePose1, squarePose2))
                .setVelocityConstraint(6)
                .setLinearHeadingInterpolation(squarePose1.getHeading(), squarePose2.getHeading())
                .build();

        pathSquare3 = fol.pathBuilder()
                .addPath(new BezierLine(squarePose2, squarePose3))
                .setLinearHeadingInterpolation(squarePose2.getHeading(), squarePose3.getHeading())
                .build();

        pathSquare4 = fol.pathBuilder()
                .addPath(new BezierLine(squarePose3, squarePose4))
                .setLinearHeadingInterpolation(squarePose3.getHeading(), squarePose4.getHeading())
                .build();
    }



}
