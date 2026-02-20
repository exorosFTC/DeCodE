package org.firstinspires.ftc.teamcode.CommandBase.Constants;

import org.firstinspires.ftc.teamcode.CustomPathing.Math.Geometry.Point;
import org.firstinspires.ftc.teamcode.CustomPathing.Math.Geometry.Pose;

public class DriveConstants {
    public static final Pose startPoseRedFar = new Pose(-160, -48, Math.toRadians(0));
    public static final Pose startPoseRedClose = new Pose(149, -107, Math.toRadians(-45));
    public static final Pose startPoseBlueFar = new Pose(-160, 48, Math.toRadians(0));
    public static final Pose startPoseBlueClose = new Pose(149, 107, Math.toRadians(45));

    public static Pose startPose = new Pose(0, 0, Math.toRadians(270));
    public static Pose POSE = startPose;
    public static Pose VELOCITY = new Pose();

    public static double VEL_X_MULTIPLIER = 0.4,
                         VEL_Y_MULTIPLIER = 0.5;

    public static double DRIVE_W = 33, DRIVE_L = 33;
    public static double R = Math.hypot(DRIVE_W, DRIVE_L);

    public static double ODOMETRY_X_OFFSET = -12.1,
                         ODOMETRY_Y_OFFSET = 6.2;

    // teleop PID
    public static double TeleOpAngularP = 0.9,
                                TeleOpAngularD = 0;
    public static double TeleOpLimelightP = 0.03,
                                TeleOpLimelightD = 0;
    public static double TeleOpVelocityMultiplier = 0;

    // auto PID
    public static double AutoLinearPx = 0.04,
                            AutoLinearDx = 0.005, //0.005,
                            AutoLinearPy = 0.04,
                            AutoLinearDy = 0.005;
    public static double AutoAngularP = 0.5, //0.6,
                            AutoAngularD = 0; //0.005;
    public static double AutoAngularVelocityMultiplier = 0;

    // swerve module PID
    public static final double TeleOpSwerveModuleP = 0.3,
                                TeleOpSwerveModuleD = 0.22;
    public static final double AutoSwerveModuleP = 0.2,
                                AutoSwerveModuleD = 0.2;


    public static double tileLengthCM = 60.96;
    public static final Point goalPositionRed = new Point(2.5 * tileLengthCM, -2.5 * tileLengthCM);
    public static final Point goalPositionBlue = new Point(2.5 * tileLengthCM, 2.5 * tileLengthCM);

    public static Point goalPosition = goalPositionRed;
}
