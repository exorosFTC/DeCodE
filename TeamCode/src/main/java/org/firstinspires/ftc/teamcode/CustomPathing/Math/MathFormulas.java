package org.firstinspires.ftc.teamcode.CustomPathing.Math;

import org.firstinspires.ftc.teamcode.CustomPathing.Math.Geometry.Point;
import org.firstinspires.ftc.teamcode.CustomPathing.Math.Geometry.Pose;

public class MathFormulas {
    public static double FindShortestPath(double currentAngle, double targetAngle) {
        double error = targetAngle - currentAngle;
        double errorAbs = Math.abs(error);

        if (errorAbs <= 2 * Math.PI - errorAbs)
            return -error;
        return Math.signum(error) * 2 * Math.PI - error;
    }



    public static double exp(double x) {
        return x * x * x;
    }



    public static Point midPoint(Point p1, Point p2) {
        return new Point((p1.x + p2.x) / 2, (p1.y + p2.y) / 2);
    }

    public static Pose midPoint(Pose p1, Pose p2, double head) {
        return new Pose((p1.x + p2.x) / 2, (p1.y + p2.y) / 2, head);
    }



    // mod normalize [-180, 180]
    public static double normalizeRadians(double angle){
        angle %= 2.0 * Math.PI;

        if(angle < -Math.PI) angle += 2.0 * Math.PI;
        if(angle > Math.PI) angle -= 2.0 * Math.PI;

        return angle;
    }

    // mod normalize [0, 360]
    public static double normalizeRadiansPositive(double angle) {
        angle %= 2.0 * Math.PI;

        if(angle < 0) angle += 2.0 * Math.PI;
        if(angle > 2.0 * Math.PI) angle -= 2.0 * Math.PI;

        return angle;
    }
}
