package org.firstinspires.ftc.teamcode.Hardware.Generals.Constants;

import static org.firstinspires.ftc.teamcode.Pathing.Math.Transformations.toIN;

import org.firstinspires.ftc.teamcode.Pathing.Math.Point;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

public class MecanumConstants {
    public static boolean usingAcceleration = true;
    public static boolean usingExponentialInput = true;
    public static boolean usingFieldCentric = false;

    public static double accelerationScalar = 0.08;


    public static final Pose basketPose = new Pose(3.1, 16.6, Math.toRadians(360 - 55));


    public static double driveSensitivity = 0.6;

    public static final double LinearP = 0.1,
                        LinearD = 0.000000004;
    public static final double AngularP = 0.3,
                        AngularD = 0.02;


    // ODOMETRY CONSTANTS
    public static final double ODOMETRY_TICKS_PER_REVOLUTION = 2000;
    public static final double ODOMETRY_WHEEL_RADIUS_CM = 3.2;

    public static double ahhX = 2000, ahhY = 0;

    public static final Point forward = new Point(2, 0.9286);       //cm
    public static final Point perpendicular = new Point(2.581, -1.7482); //cm

    public static final double TRACK_WIDTH = 5; //23 og, but it's turning too fast
    public static final double ODOMETRY_X_MULTIPLIER = 0.2;
    public static final double ODOMETRY_Y_MULTIPLIER = 0.2;


}
