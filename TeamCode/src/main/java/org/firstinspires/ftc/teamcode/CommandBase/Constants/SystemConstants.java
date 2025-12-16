package org.firstinspires.ftc.teamcode.CommandBase.Constants;

import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.ShotSample;

public class SystemConstants {

    public static Enums.OpMode opModeType;

    public static boolean telemetryAddLoopTime = false;
    public static boolean autoOnBlue = false;

    public static List<ShotSample> samples = Arrays.asList(
            new ShotSample(398, 0.95, 0.65),
            new ShotSample(381, 0.91, 0.65),
            new ShotSample(371, 0.9, 0.69),
            new ShotSample(361, 0.9, 0.64),
            new ShotSample(357, 0.9, 0.79),
            new ShotSample(349, 0.9, 0.79),
            new ShotSample(343, 0.9, 0.74),
            new ShotSample(320, 0.9, 0.74),
            new ShotSample(313, 0.9, 0.64),

            new ShotSample(246, 0.73, 0.39),
            new ShotSample(195, 0.7, 0.2)
    );
}
