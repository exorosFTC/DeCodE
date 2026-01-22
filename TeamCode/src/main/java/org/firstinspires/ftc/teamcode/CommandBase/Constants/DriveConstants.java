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

    public static double TELE_OP_STRAFING_SLEW_RATE_LIMIT = 7,
                        TELE_OP_TURNING_SLEW_RATE_LIMIT = Double.POSITIVE_INFINITY;
    public static double AUTO_STRAFING_SLEW_RATE_LIMIT = 3,
                        AUTO_TURNING_SLEW_RATE_LIMIT = 5.67;


    // teleop PID
    public static double TeleOpAngularP = 0,
                                TeleOpAngularD = 0;
    public static double TeleOpLimelightP = 0,
                                TeleOpLimelightD = 0;
    public static double TeleOpVelocityMultiplier = 0.3;

    // auto PID
    public static double AutoLinearPx = 0.012,
                            AutoLinearDx = 0,
                            AutoLinearPy = 0.012,
                            AutoLinearDy = 0;
    public static double AutoAngularP = 0,
                            AutoAngularD = 0;
    public static double AutoAngularVelocityMultiplier = 0.3,
                            AutoLinearVelocityMultiplier = 0;

    // swerve module PID
    public static final double swerveModuleP = 0.3,
                                swerveModuleD = 0.03;


    public static double tileLengthCM = 60.96;
    public static final Point goalPositionRed = new Point(2.5 * tileLengthCM, -2.5 * tileLengthCM);
    public static final Point goalPositionBlue = new Point(2.5 * tileLengthCM, 2.5 * tileLengthCM);

    public static Point goalPosition = goalPositionRed;
}
