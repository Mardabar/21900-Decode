package org.firstinspires.ftc.teamcode.paths.tankPaths;



import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class TankFigure8Paths {
    public final Pose startPose = new Pose(56, 47, Math.toRadians(0));

    public final Pose squarePose1 = new Pose(100, 47, Math.toRadians(0));
    public final Pose squarePose2 = new Pose(100, 90, Math.toRadians(90));
    public final Pose squarePose3 = new Pose(56, 90, Math.toRadians(180));
    public final Pose squarePose4 = new Pose(56, 47, Math.toRadians(270));

    public final Pose f8Pose1 = new Pose(56, 87), f8Pose1CP = new Pose(123, 91);
    public final Pose f8Pose2 = new Pose(90, 48), f8Pose2CP = new Pose(51, 65);
    public final Pose f8Pose3 = new Pose(56,47), f8Pose3CP = new Pose(88, 77);

    public PathChain path1, path2, path3, path4;

    public TankFigure8Paths(Follower fol) {

        path1 = fol.pathBuilder()
                .addPath(new BezierCurve(startPose, f8Pose1CP, f8Pose1))
                .setTangentHeadingInterpolation()
                .build();

        path2 = fol.pathBuilder()
                .addPath(new BezierCurve(f8Pose1, f8Pose2CP, f8Pose2))
                .setTangentHeadingInterpolation()
                .build();

        path3 = fol.pathBuilder()
                .addPath(new BezierCurve(f8Pose2, f8Pose3CP, f8Pose3))
                .setTangentHeadingInterpolation()
                .build();

    }
}

