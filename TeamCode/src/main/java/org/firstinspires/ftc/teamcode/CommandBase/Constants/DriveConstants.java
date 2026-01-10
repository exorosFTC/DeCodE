package org.firstinspires.ftc.teamcode.CommandBase.Constants;

import org.firstinspires.ftc.teamcode.Pathing.Math.Point;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

public class DriveConstants {
    public static final Pose startPoseRedFar = new Pose(-186, -48, Math.toRadians(0));
    public static final Pose startPoseRedClose = new Pose(149, -107, Math.toRadians(45));
    public static final Pose startPoseBlueFar = new Pose(-186, 48, Math.toRadians(0));
    public static final Pose startPoseBlueClose = new Pose(149, 107, Math.toRadians(315));

    public static Pose startPose = new Pose(0, 0, Math.toRadians(270));
    public static Pose POSE = startPose;

    public static double DRIVE_W = 33, DRIVE_L = 33;
    public static double R = Math.hypot(DRIVE_W, DRIVE_L);

    public static double ODO_UPDATE_RATE_AUTO = 100,
                        ODO_UPDATE_RATE_TELEOP = 10;
    public static double STRAFING_SLEW_RATE_LIMIT = 3,
                        TURNING_SLEW_RATE_LIMIT = 5.67;


    // teleop PID
    public static final double TeleOpAngularP = 0.55,
                                TeleOpAngularD = 0;
    public static final double TeleOpLimelightP = 0,
                                TeleOpLimelightD = 0;

    // auto PID
    public static final double AutoLinearPx = 0.019,
                                AutoLinearDx = 0,
                                AutoLinearPy = 0.019,
                                AutoLinearDy = 0;
    public static final double AutoAngularP = 0.55,
                                AutoAngularD = 0.0;

    // swerve module PID
    public static final double swerveModuleP = 0.21,  // 0.21
                                swerveModuleD = 0.16; // 0.16
    public static double K_STATIC = 0.045;



    public static double tileLengthCM = 60.96;
    public static final Point goalPositionRed = new Point(2.5 * tileLengthCM, -2.5 * tileLengthCM);
    public static final Point goalPositionBlue = new Point(2.5 * tileLengthCM, 2.5 * tileLengthCM);

    public static Point goalPosition = goalPositionRed;
}
