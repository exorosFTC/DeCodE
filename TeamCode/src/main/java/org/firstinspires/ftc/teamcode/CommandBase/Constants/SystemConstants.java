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
            new ShotSample(80, 0.44, 0.94, 1),
            new ShotSample(94, 0.46, 0.94, 1),
            new ShotSample(115, 0.49, 0.74, 1),
            new ShotSample(139, 0.53, 0.69, 1),
            new ShotSample(169, 0.55, 0.65, 0.9),
            new ShotSample(197, 0.58, 0.57, 0.8),
            new ShotSample(214, 0.60, 0.44, 0.7),
            new ShotSample(286, 0.67, 0.39, 0.5),

            new ShotSample(338, 0.750, 0.37, 0.4),
            new ShotSample(363, 0.760, 0.35, 0.4)
    );
}
