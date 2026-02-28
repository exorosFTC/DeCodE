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

    public static boolean autoOnBlue = false;
    public static boolean aBitOfTrolling = true;

    public static List<ShotSample> samples = Arrays.asList(
            new ShotSample(360, 0.72, 0.39, 0.5),
            new ShotSample(321, 0.65, 0.45, 0.54),

            new ShotSample(281, 0.58, 0.45, 0.65),
            new ShotSample(213, 0.56, 0.7, 0.75),
            new ShotSample(170, 0.54, 0.6, 1),
            new ShotSample(112, 0.472, 0.8, 1),
            new ShotSample(83, 0.47, 0.9, 1)
    );
}
