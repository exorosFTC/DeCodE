package org.firstinspires.ftc.teamcode.CommandBase.Constants;

import org.firstinspires.ftc.teamcode.CustomPathing.Math.Geometry.Point;
import org.firstinspires.ftc.teamcode.CustomPathing.Math.Geometry.Pose;

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
    public static double STRAFING_SLEW_RATE_LIMIT = 2.5,
                        TURNING_SLEW_RATE_LIMIT = 5.67;


    // teleop PID
    public static double TeleOpAngularP = 0,
                                TeleOpAngularD = 0;
    public static double TeleOpLimelightP = 0,
                                TeleOpLimelightD = 0;
    public static double TeleOpVelocityMultiplier = 0.45;

    // auto PID
    public static double AutoLinearPx = 0.02,
                                AutoLinearDx = 0.0000001,
                                AutoLinearPy = 0.02,
                                AutoLinearDy = 0.0000001;
    public static double AutoAngularP = 0.9,
                                AutoAngularD = 0.27;
    public static double AutoAngularVelocityMultiplier = 0.45,
                        AutoLinearVelocityMultiplier = 0;

    // swerve module PID
    public static final double swerveModuleP = 0.2,
                                swerveModuleD = 0.15;


    public static double tileLengthCM = 60.96;
    public static final Point goalPositionRed = new Point(2.5 * tileLengthCM, -2.5 * tileLengthCM);
    public static final Point goalPositionBlue = new Point(2.5 * tileLengthCM, 2.5 * tileLengthCM);

    public static Point goalPosition = goalPositionRed;
}
