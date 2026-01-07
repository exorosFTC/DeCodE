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
            new ShotSample(80, 0.58, 0.97),
            new ShotSample(94, 0.6, 0.97),
            new ShotSample(115, 0.61, 0.85),
            new ShotSample(139, 0.62, 0.8),
            new ShotSample(169, 0.64, 0.76),
            new ShotSample(197, 0.73, 0.58),
            new ShotSample(214, 0.76, 0.55),
            new ShotSample(286, 0.8, 0.5),

            new ShotSample(338, 0.95, 0.45),
            new ShotSample(363, 0.99, 0.435)
    );
}
