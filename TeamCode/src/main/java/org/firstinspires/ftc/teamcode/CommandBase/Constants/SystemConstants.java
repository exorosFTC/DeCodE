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
            new ShotSample(80, 0.55, 0.94),
            new ShotSample(94, 0.57, 0.94),
            new ShotSample(115, 0.58, 0.82),
            new ShotSample(139, 0.59, 0.77),
            new ShotSample(169, 0.61, 0.73),
            new ShotSample(197, 0.67, 0.55),
            new ShotSample(214, 0.70, 0.52),
            new ShotSample(286, 0.74, 0.47),

            new ShotSample(338, 0.86, 0.37),
            new ShotSample(363, 0.91, 0.34)
    );
}
