package org.firstinspires.ftc.teamcode.CommandBase.Constants;

import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.ShotSample;

public class SystemConstants {

    public static Enums.Randomization lastValidRandomization = Enums.Randomization.LEFT;
    public static Enums.OpMode opModeType;

    public static boolean telemetryAddLoopTime = false;
    public static boolean autoOnBlue = false;

    public static List<ShotSample> samples = Arrays.asList(
            new ShotSample(80, 0.59, 0.97),
            new ShotSample(94, 0.61, 0.97),
            new ShotSample(115, 0.62, 0.85),
            new ShotSample(139, 0.63, 0.8),
            new ShotSample(169, 0.65, 0.76),
            new ShotSample(197, 0.71, 0.58),
            new ShotSample(214, 0.74, 0.55),
            new ShotSample(286, 0.78, 0.5),

            new ShotSample(338, 0.90, 0.35),
            new ShotSample(363, 0.95, 0.3)
    );
}
