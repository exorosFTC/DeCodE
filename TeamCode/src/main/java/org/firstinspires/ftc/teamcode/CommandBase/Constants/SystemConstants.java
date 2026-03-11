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
            new ShotSample(365, 0.689, 0.35, 0.4),
            new ShotSample(331, 0.674, 0.37, 0.4),
            new ShotSample(302, 0.66, 0.39, 0.4),

            new ShotSample(248, 0.63, 0.42, 0.85),
            new ShotSample(215, 0.595, 0.5, 0.85),
            new ShotSample(191, 0.57, 0.5, 0.9),
            new ShotSample(169, 0.535, 0.67, 0.95),
            new ShotSample(125, 0.47, 0.72, 1),
            new ShotSample(97, 0.46, 0.85, 1),
            new ShotSample(74, 0.45, 0.97, 1)

    );
}
