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
            new ShotSample(115, 0.49, 0.82, 1),
            new ShotSample(139, 0.53, 0.77, 1),
            new ShotSample(169, 0.56, 0.73, 0.9),
            new ShotSample(197, 0.60, 0.55, 0.8),
            new ShotSample(214, 0.62, 0.52, 0.7),
            new ShotSample(286, 0.66, 0.47, 0.5),

            new ShotSample(338, 0.730, 0.37, 0.4),
            new ShotSample(363, 0.755, 0.35, 0.4)
    );
}
