package org.firstinspires.ftc.teamcode.Hardware.Constants;

import org.firstinspires.ftc.teamcode.Pathing.Math.Point;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

public class DriveConstants {
    public static double RANGE = 3.3;
    public static boolean VALUE_REJECTION = false;

    public static boolean usingAcceleration = false;
    public static boolean usingExponentialInput = false;
    public static boolean usingFieldCentric = true;


    public static double DRIVE_W = 9, DRIVE_L = 9; //to be tuned
    public final double R = Math.hypot(DRIVE_W, DRIVE_L) / 2;


    public static double accelerationScalar = 0.08;
    public static double driveSensitivity = 0.5;



    // auto PID
    public static final double LinearP = 0.1,
                                LinearD = 0.000000004;
    public static final double AngularP = 0.1,
                                AngularD = 0;

    // swerve PID
    public static final double swerveP = 0.5,
                                swerveD = 0.1;
    public static double K_STATIC = 0.03;



    public static double tileLengthCM = 60.96;
    public static Point redGoalPosition = new Point(2.7 * tileLengthCM, -2.7 * tileLengthCM);
    public static Point blueGoalPosition = new Point(2.7 * tileLengthCM, 2.7 * tileLengthCM);

}
