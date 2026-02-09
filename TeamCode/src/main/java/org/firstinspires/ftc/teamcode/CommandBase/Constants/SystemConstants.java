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

    public static List<ShotSample> samples = Arrays.asList(
            new ShotSample(80, 0.54, 0.94, 1),
            new ShotSample(94, 0.56, 0.94, 1),
            new ShotSample(115, 0.57, 0.82, 1),
            new ShotSample(139, 0.58, 0.77, 1),
            new ShotSample(169, 0.60, 0.73, 1),
            new ShotSample(197, 0.66, 0.55, 1),
            new ShotSample(214, 0.69, 0.52, 1),
            new ShotSample(286, 0.73, 0.47, 0.8),

            new ShotSample(338, 0.835, 0.37, 0.55),
            new ShotSample(363, 0.84, 0.35, 0.568)
    );
}
