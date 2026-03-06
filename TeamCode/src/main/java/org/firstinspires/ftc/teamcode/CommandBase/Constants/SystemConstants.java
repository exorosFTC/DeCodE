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

    public static boolean soloDrive = false;
    public static boolean autoOnBlue = false;
    public static boolean aBitOfTrolling = true;

    public static List<ShotSample> samples = Arrays.asList(

            new ShotSample(362, 0.78, 0.35, 0.5),
            new ShotSample(322, 0.7, 0.35, 0.5),
            new ShotSample(296, 0.67, 0.38, 0.5),

            new ShotSample(245, 0.59, 0.53, 1),
            new ShotSample(210, 0.568, 0.6, 1),
            new ShotSample(159, 0.527, 0.67, 1),
            new ShotSample(124, 0.50, 0.73, 1),
            new ShotSample(94, 0.48, 0.84, 1),
            new ShotSample(72, 0.472, 0.97, 1)
    );
}
