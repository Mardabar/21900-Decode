package org.firstinspires.ftc.teamcode.paths.tankPaths;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class TankSquarePaths {
    public final Pose startPose = new Pose(56, 47, Math.toRadians(0));

    public final Pose squarePose1 = new Pose(100, 47);
    public final Pose squarePose2 = new Pose(100, 90);
    public final Pose squarePose3 = new Pose(56, 90);
    public final Pose squarePose4 = new Pose(56, 47);

    public PathChain path1, path2, path3, path4;

    public TankSquarePaths(Follower fol) {
        path1 = fol.pathBuilder()
                .addPath(new BezierLine(startPose, squarePose1))
                .setTangentHeadingInterpolation()
                .build();

        path2 = fol.pathBuilder()
                .addPath(new BezierLine(squarePose1, squarePose2))
                .setTangentHeadingInterpolation()
                .build();

        path3 = fol.pathBuilder()
                .addPath(new BezierLine(squarePose2, squarePose3))
                .setTangentHeadingInterpolation()
                .build();

        path4 = fol.pathBuilder()
                .addPath(new BezierLine(squarePose3, squarePose4))
                .setTangentHeadingInterpolation()
                .build();
    }
}
