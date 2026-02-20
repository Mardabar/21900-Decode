package org.firstinspires.ftc.teamcode.config.subsystems;
//

import com.pedropathing.geometry.Pose;

public final class MathUtilities {
    public static double expo(double input)
    {
        return Math.signum(input) * Math.pow(Math.abs(input), 1.4);
    }


    public static double calculateDistance(Pose currentPose, Pose targetPose) {
        return Math.hypot(targetPose.getX() - currentPose.getX(), targetPose.getY() - currentPose.getY());
    }
    public static double[] calcDist(double x1, double y1, double x2, double y2){
        double deltaX = Math.abs(x2 - x1);
        double deltaY = Math.abs(y2 - y1);
        return new double[] {deltaX, deltaY};
    }

}
