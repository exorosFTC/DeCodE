package org.firstinspires.ftc.teamcode.Hardware.Generals.Constants;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeDegrees;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Util.SensorsEx.ColorEx;

public class SystemConstants {

    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    public static RevHubOrientationOnRobot.UsbFacingDirection  USB_DIRECTION  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

    public static Enums.Color detectionColor = Enums.Color.YELLOW;
    public static Enums.OpMode opModeType;



    public static double manualLiftCoefficient = 1;
    public static final int outtakeMAX = 920;
    public static int extendoMAX = 830;

    public static final int intakeFullRetractTimeMs = 750;
    public static final int outtakeFullRetractTimeMs = 300;

    public static final double isExtensionOut = 200;
    public static final double inIntakeThreshold = 10; //mm

    public static boolean updateOuttake = true;


    public static double waitReachedIntake = 1;
    public static double waitReachedOuttake = 6;
    public static boolean successfulCatch = false;




    public static double turretMin = 0; // 0
    public static double turretMax = 1; // 355

    public static final double turretMaxWristUp = 0.5;
    public static final double turretMaxIntakeRetracted = 0.1;
    public static final double turretMaxIntakeExtended = 0.25;




    // color thresholding only by red values
    public static final ColorEx red = new ColorEx(5000, 2670, 1717);
    public static final ColorEx blue = new ColorEx(1027, 2000, 5420);
    public static final ColorEx yellow = new ColorEx(9670, 11150, 2295);

    public static boolean ENCODER_VALUE_REJECTION = true;

    public static boolean telemetryAddLoopTime = false;
    public static boolean autoOnBlue = false;

    public static boolean usingOpenCvCamera = false;
    public static boolean usingAprilTagCamera = false;

    public static boolean multithreading = true;

    public static Enums.Randomization randomization = Enums.Randomization.CENTER;



}
