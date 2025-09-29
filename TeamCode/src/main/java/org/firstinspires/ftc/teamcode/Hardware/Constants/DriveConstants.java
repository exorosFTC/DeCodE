package org.firstinspires.ftc.teamcode.Hardware.Constants;

import org.firstinspires.ftc.teamcode.Pathing.Math.Point;

public class DriveConstants {
    public static double RANGE = 3.3;
    public static boolean VALUE_REJECTION = false;

    public static boolean usingAcceleration = true;
    public static boolean usingExponentialInput = true;
    public static boolean usingFieldCentric = false;
    public static boolean useMotorFlipping = true;


    public static double DRIVE_W = 9, DRIVE_L = 9; //to be tuned
    public final double R = Math.hypot(DRIVE_W, DRIVE_L) / 2;


    public static double accelerationScalar = 0.08;
    public static double driveSensitivity = 0.6;



    // auto PID
    public static final double LinearP = 0.1,
                                LinearD = 0.000000004;
    public static final double AngularP = 0.3,
                                AngularD = 0.02;

    // swerve PID
    public static final double swerveP = 0.1,
                                swerveD = 0;



    // ODOMETRY CONSTANTS
    public static final double ODOMETRY_TICKS_PER_REVOLUTION = 2000;
    public static final double ODOMETRY_WHEEL_RADIUS_CM = 3.2;

    public static double ahhX = 2000, ahhY = 0;

    public static final Point forward = new Point(0, 0);       //cm
    public static final Point perpendicular = new Point(0, 0); //cm

    public static final double TRACK_WIDTH = 5; //23 og, but it's turning too fast
    public static final double ODOMETRY_X_MULTIPLIER = 0.2;
    public static final double ODOMETRY_Y_MULTIPLIER = 0.2;
}
