package org.firstinspires.ftc.teamcode.Hardware.Generals;

import java.util.Arrays;
import java.util.List;


public class HardwareNames {
    public static final String cameraConfigurationName = "ExoCamera";
    public static final String IMU_Name = "ExoIMU";



    // TODO: modify your configuration names
    public static final String LeftFront = "LF";
    public static final String LeftBack = "LB";
    public static final String RightFront = "RF";
    public static final String RightBack = "RB";

    public static final String IntakeExtensionMotor = "intakeExtensionMotor";
    public static final String IntakeMotor = "intakeCollectionMotor";
    public static final String RightOuttakeMotor = "rightOuttakeMotor";
    public static final String LeftOuttakeMotor = "leftOuttakeMotor";



    // you can leave these as they are
    public static final List<String> MotorNamesList = Arrays.asList(
            LeftFront, LeftBack, RightFront, RightBack,
            IntakeExtensionMotor, IntakeMotor,
            RightOuttakeMotor, LeftOuttakeMotor);



    // TODO: tune these
    // If you use 2 or no odometry at all, set the names to "" (empty string)
    public static final String LeftOdometry = LeftBack;
    public static final String RightOdometry = "";
    public static final String PerpendicularOdometry = RightBack;

    // you can leave these as they are
    public static final List<String> EncoderNamesList = Arrays.asList(
            LeftOdometry, PerpendicularOdometry
    );



    // TODO: add additional hardware components in the lists below
    public static final String IntakeTurret = "intakeTurret";
    public static final String IntakeWrist = "intakeWrist";
    public static final String IntakeLocker = "intakeLocker";

    public static final String OuttakeLeftPivot = "outtakeLeft";
    public static final String OuttakeRightPivot = "outtakeRight";
    public static final String OuttakeWrist = "outtakeWrist";
    public static final String OuttakeClaw = "outtakeClaw";

    public static List<String> ServoNamesList = Arrays.asList(
            IntakeTurret, IntakeWrist, IntakeLocker,
            OuttakeLeftPivot, OuttakeRightPivot, OuttakeWrist, OuttakeClaw
    );




    public static List<String> CRServoNamesList = Arrays.asList("", "");
    public static List<String> DigitalNamesList = Arrays.asList("", "");
    public static List<String> RevDistanceNameList = Arrays.asList("", "");
    public static List<String> RevTouchNameList = Arrays.asList("", "");


    public static final String LeftUltrasonic = "left";
    public static final String RightUltrasonic = "right";
    public static final String FrontUltrasonic = "front";

    public static List<String> AnalogNamesList = Arrays.asList(
            LeftUltrasonic, RightUltrasonic, FrontUltrasonic
    );




    public static final String IntakeColor = "intakeColorSensor";

    public static List<String> RevColorNameList = Arrays.asList(
        IntakeColor
    );





}
