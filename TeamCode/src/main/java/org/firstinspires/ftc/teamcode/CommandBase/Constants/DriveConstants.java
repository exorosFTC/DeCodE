package org.firstinspires.ftc.teamcode.CommandBase.Constants;

import org.firstinspires.ftc.teamcode.Pathing.Math.Point;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

public class DriveConstants {
    public static double RANGE = 3.3;

    public static Pose startPose = new Pose(0, 0, Math.toRadians(270));
    public static Pose POSE = startPose;

    public static boolean usingFieldCentric = true;
    public static double DRIVE_W = 9, DRIVE_L = 9;


    // auto PID
    public static final double LinearP = 0.1,
                                LinearD = 0.000000004;
    public static final double AngularP = 0.1,
                                AngularD = 0.001;

    // swerve module PID
    public static final double swerveP = 0.4,
                                swerveD = 0.001;
    public static double K_STATIC = 0.03;



    public static double tileLengthCM = 60.96;
    public static Point goalPosition = new Point(2.5 * tileLengthCM, -2.5 * tileLengthCM);
}
