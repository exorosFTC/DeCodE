package org.firstinspires.ftc.teamcode.Pathing.Math;

import java.util.Arrays;

import javax.annotation.Nullable;

public class MathFormulas {
    public static double FindShortestPath(double currentAngle, double targetAngle) {
        double error = targetAngle - currentAngle;
        double errorAbs = Math.abs(error);

        if (errorAbs <= 2 * Math.PI - errorAbs)
            return -error;
        return sign(error) * 2 * Math.PI - error;
    }

    public static double sign(double value) { return (value < 0) ? -1 : 1; }

    public static double clip(double val, double min, double max) {
        if (val < min) return min;
        if (val > max) return max;

        return val;
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




}
