package org.firstinspires.ftc.teamcode.Hardware.Constants;

import org.firstinspires.ftc.teamcode.Pathing.Math.Point;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

public class DriveConstants {
    public static double RANGE = 3.3;

    public static Pose startPose = new Pose(0, 0, Math.toRadians(270));
    public static Pose POSE = startPose;

    public static boolean usingFieldCentric = true;


    public static double DRIVE_W = 9, DRIVE_L = 9; //to be tuned


    public static double accelerationScalar = 0.08;


    // auto PID
    public static final double LinearP = 0.1,
                                LinearD = 0.000000004;
    public static final double AngularP = 0.08,
                                AngularD = 0;

    // swerve PID
    public static final double swerveP = 0.5,
                                swerveD = 0.1;
    public static double K_STATIC = 0.03;



    public static double tileLengthCM = 60.96;
    public static Point goalPosition = new Point(3.2 * tileLengthCM, -3 * tileLengthCM);
}
