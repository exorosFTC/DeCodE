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




}
