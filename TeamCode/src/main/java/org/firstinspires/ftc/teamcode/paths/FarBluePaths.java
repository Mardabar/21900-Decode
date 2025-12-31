package org.firstinspires.ftc.teamcode.paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
public class FarBluePaths {

    static public Follower fol;

    private static final Pose Start = new Pose(56, 9, Math.toRadians(90)); // STARTING POSITION
    private final Pose PreScore = new Pose(60, 22, Math.toRadians(116)); // PRE-LOAD SCORING POSITION
    private final Pose Grab1Set = new Pose(45, 34.4, Math.toRadians(0)); // POSITION
    private final Pose Grab1 = new Pose(20, 34.4, Math.toRadians(0)); // POSITION
    private final Pose Score1 = new Pose(60, 75, Math.toRadians(133.5)); // POSITION
    private final Pose Score1CP = new Pose(60, 34.4, Math.toRadians(133.5)); // CONTROL POINT
    private final Pose Grab2Set = new Pose(45, 60, Math.toRadians(0)); // POSITION
    private final Pose Grab2 = new Pose(31, 60, Math.toRadians(0)); // POSITION
    private final Pose Score2 = new Pose(60, 75, Math.toRadians(139)); // POSITION
    private final Pose Grab3Set = new Pose(45, 84, Math.toRadians(0)); // POSITION
    private final Pose Grab3 = new Pose(31, 84, Math.toRadians(0)); // POSITION
    private final Pose Score3 = new Pose(60, 75, Math.toRadians(136)); // POSITION
    private final Pose parkPose = new Pose(50, 65, Math.toRadians(139)); // PARKING POSITION

    private PathChain pathPreScore, pathGrab1Set, pathGrab1, pathScore1, pathGrab2Set, pathGrab2,
            pathScore2, pathGrab3Set, pathGrab3, pathScore3, pathPark;




    public FarBluePaths(Follower fol){

        pathPreScore = fol.pathBuilder()
                .addPath(new BezierLine(Start, PreScore))
                .setLinearHeadingInterpolation(Start.getHeading(), PreScore.getHeading())
                .setBrakingStrength(4)
                .build();

        pathGrab1Set = fol.pathBuilder()
                .addPath(new BezierLine(PreScore, Grab1Set))
                .setLinearHeadingInterpolation(PreScore.getHeading(), Grab1Set.getHeading())
                .build();

        pathGrab1 = fol.pathBuilder()
                .addPath(new BezierLine(Grab1Set, Grab1))
                .setLinearHeadingInterpolation(Grab1Set.getHeading(), Grab1.getHeading())
                .build();

        pathScore1 = fol.pathBuilder()
                .addPath(new BezierCurve(Grab1, Score1CP, Score1))
                .setLinearHeadingInterpolation(Grab1.getHeading(), Score1.getHeading())
                .build();

        pathGrab2Set = fol.pathBuilder()
                .addPath(new BezierLine(Score1, Grab2Set))
                .setLinearHeadingInterpolation(Score1.getHeading(), Grab2Set.getHeading())
                .build();

        pathGrab2 = fol.pathBuilder()
                .addPath(new BezierLine(Grab2Set, Grab2))
                .setLinearHeadingInterpolation(Grab2Set.getHeading(), Grab2.getHeading())
                .build();

        pathScore2 = fol.pathBuilder()
                .addPath(new BezierLine(Grab2, Score2))
                .setLinearHeadingInterpolation(Grab2.getHeading(), Score2.getHeading())
                .build();

        pathGrab3Set = fol.pathBuilder()
                .addPath(new BezierLine(Score2, Grab3Set))
                .setLinearHeadingInterpolation(Score2.getHeading(), Grab3Set.getHeading())
                .build();

        pathGrab3 = fol.pathBuilder()
                .addPath(new BezierLine(Grab3Set, Grab3))
                .setLinearHeadingInterpolation(Grab3Set.getHeading(), Grab3.getHeading())
                .build();

        pathScore3 = fol.pathBuilder()
                .addPath(new BezierLine(Grab3, Score3))
                .setLinearHeadingInterpolation(Grab3.getHeading(), Score3.getHeading())
                .build();

        pathPark = fol.pathBuilder()
                .addPath(new BezierLine(Score2, parkPose))
                .setLinearHeadingInterpolation(Score2.getHeading(), parkPose.getHeading())
                .build();

    }

}
