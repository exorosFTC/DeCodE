package org.firstinspires.ftc.teamcode.CommandBase.Constants;

import org.firstinspires.ftc.teamcode.Pathing.Math.Point;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

public class DriveConstants {
    public static final Pose startPoseRedFar = new Pose(-186, -48, Math.toRadians(0));
    public static final Pose startPoseRedClose = new Pose(149, -107, 2.3694);
    public static final Pose startPoseBlueFar = new Pose(-186, 48, Math.toRadians(0));
    public static final Pose startPoseBlueClose = new Pose(149, 107, 3.9438);

    public static Pose startPose = new Pose(0, 0, Math.toRadians(270));
    public static Pose POSE = startPose;

    public static double DRIVE_W = 33, DRIVE_L = 33;

    public static double ODO_UPDATE_RATE_AUTO = 100,
                        ODO_UPDATE_RATE_TELEOP = 10;
    public static double STRAFING_SLEW_RATE_LIMIT = 8.7,
                        TURNING_SLEW_RATE_LIMIT = 5.67;


    // teleop PID
    public static final double TeleOpAngularP = 0.02,
                                TeleOpAngularD = 0.004;
    public static final double TeleOpLimelightP = 0,
                                TeleOpLimelightD = 0;

    // auto PID
    public static final double AutoLinearP = 0,
                                AutoLinearD = 0;

    // swerve module PID
    public static final double swerveP = 0.2,
                                swerveD = 0;
    public static double K_STATIC = 0.03;



    public static double tileLengthCM = 60.96;
    public static final Point goalPositionRed = new Point(2.5 * tileLengthCM, -2.5 * tileLengthCM);
    public static final Point goalPositionBlue = new Point(2.5 * tileLengthCM, 2.5 * tileLengthCM);

    public static Point goalPosition = goalPositionRed;
}
