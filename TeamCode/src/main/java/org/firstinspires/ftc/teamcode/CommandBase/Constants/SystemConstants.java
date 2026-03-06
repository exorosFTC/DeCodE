package org.firstinspires.ftc.teamcode.CommandBase.Constants;

import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.ShotSample;
import org.firstinspires.ftc.teamcode.CommandBase.Util.SensorsEx.LimelightEx;

public class SystemConstants {
    public enum OpMode{
        TELE_OP,
        AUTONOMOUS
    }

    public static LimelightEx.Randomization lastValidRandomization = LimelightEx.Randomization.LEFT;
    public static OpMode opModeType;

    public static boolean soloDrive = false;
    public static boolean autoOnBlue = false;
    public static boolean aBitOfTrolling = true;

    public static List<ShotSample> samples = Arrays.asList(
            new ShotSample(365, 0.699, 0.35, 0.4),
            new ShotSample(331, 0.684, 0.37, 0.8),
            new ShotSample(302, 0.67, 0.39, 0.8),

            new ShotSample(248, 0.605, 0.47, 0.85),
            new ShotSample(215, 0.589, 0.5, 1),
            new ShotSample(191, 0.57, 0.55, 1),
            new ShotSample(169, 0.545, 0.67, 1),
            new ShotSample(125, 0.509, 0.72, 1),
            new ShotSample(97, 0.467, 0.85, 1),
            new ShotSample(74, 0.475, 0.97, 1)

    );
}
