package org.firstinspires.ftc.teamcode.CommandBase.Constants;

import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.ShotSample;

public class SystemConstants {

    public static Enums.OpMode opModeType;

    public static boolean telemetryAddLoopTime = false;
    public static boolean autoOnBlue = false;

    public static boolean usingOpenCvCamera = false;
    public static boolean usingAprilTagCamera = false;

    public static List<ShotSample> samples = Arrays.asList(
            new ShotSample(398, 0.95, 0.76),
            new ShotSample(381, 0.91, 0.76),
            new ShotSample(371, 0.9, 0.8),
            new ShotSample(361, 0.9, 0.75),
            new ShotSample(357, 0.9, 0.9),
            new ShotSample(349, 0.9, 0.9),
            new ShotSample(343, 0.9, 0.85),
            new ShotSample(320, 0.9, 0.85),
            new ShotSample(313, 0.9, 0.75),

            new ShotSample(246, 0.8, 0.318),
            new ShotSample(195, 0.76, 0.2)
    );

    public static Enums.Randomization randomization = Enums.Randomization.CENTER;
}
