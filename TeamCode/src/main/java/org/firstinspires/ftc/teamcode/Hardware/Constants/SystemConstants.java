package org.firstinspires.ftc.teamcode.Hardware.Constants;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class SystemConstants {
    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
    public static RevHubOrientationOnRobot.UsbFacingDirection  USB_DIRECTION  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

    public static Enums.OpMode opModeType;

    public static double manualLiftCoefficient = 1;
    public static final int outtakeMAX = 920;

    public static boolean updateOuttake = true;

    public static boolean telemetryAddLoopTime = false;
    public static boolean autoOnBlue = false;

    public static boolean usingOpenCvCamera = false;
    public static boolean usingAprilTagCamera = false;

    public static boolean multithreading = true;

    public static Enums.Randomization randomization = Enums.Randomization.CENTER;
}
