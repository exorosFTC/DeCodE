package org.firstinspires.ftc.teamcode.CommandBase.Constants;

import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.ShotSample;

public class SystemConstants {

    public static Enums.OpMode opModeType;

    public static boolean telemetryAddLoopTime = false;
    public static boolean autoOnBlue = false;

    public static List<ShotSample> samples = Arrays.asList(
            new ShotSample(94, 0.6, 0.97),
            new ShotSample(115, 0.6, 0.85),
            new ShotSample(139, 0.61, 0.8),
            new ShotSample(169, 0.63, 0.76),
            new ShotSample(197, 0.72, 0.58),
            new ShotSample(214, 0.73, 0.55),
            new ShotSample(286, 0.74, 0.5),

            new ShotSample(338, 0.95, 0.45),
            new ShotSample(365, 0.99, 0.43)
    );
}
