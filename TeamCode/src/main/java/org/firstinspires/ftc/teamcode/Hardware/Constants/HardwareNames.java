package org.firstinspires.ftc.teamcode.Hardware.Constants;

import java.util.Arrays;
import java.util.List;


public class HardwareNames {
    public static final String cameraConfigurationName = "ExoCamera";
    public static final String IMU_Name = "ExoIMU";
    public static final String PinpointName = "ExoPinpoint";



    // TODO: modify your configuration names
    public static final String LeftFront = "LF";
    public static final String LeftBack = "LB";
    public static final String RightFront = "RF";
    public static final String RightBack = "RB";

    public static final String LeftFront_servo = LeftFront + " servo";
    public static final String LeftBack_servo = LeftBack + " servo";
    public static final String RightFront_servo = RightFront + " servo";
    public static final String RightBack_servo = RightBack + " servo";

    public static final String IndexerMotor = "indexer";
    public static final String IntakeMotor = "intake";
    public static final String ShooterMotor1 = "shooter1";
    public static final String ShooterMotor2 = "shooter2";

    public static final String TiltLeftServo = "TiltL";
    public static final String TiltRightServo = "TiltR";
    public static final String ShooterHoodServo = "hood";

    public static final String BreakBeam = "breakbeam";
    public static final String IndexerLimit = "indexerLim";

    public static final String IntakeColor = "intakeColorSensor";

    public static final String LeftFront_encoder = LeftFront + " encoder";
    public static final String LeftBack_encoder = LeftBack + " encoder";
    public static final String RightFront_encoder = RightFront + " encoder";
    public static final String RightBack_encoder = RightBack + " encoder";



    // TODO: tune these
    // If you use 2 or no odometry at all, set the names to "" (empty string)
    public static final String LeftOdometry = LeftBack;
    public static final String RightOdometry = "";
    public static final String PerpendicularOdometry = RightBack;



    // unused
    public static final String RightOuttakeMotor = "rightOuttakeMotor";
    public static final String LeftOuttakeMotor = "leftOuttakeMotor";





    // TODO: add additional hardware components in the lists below

    // you can leave these as they are
    public static final List<String> MotorNamesList = Arrays.asList(
            LeftFront, LeftBack, RightFront, RightBack,
            IndexerMotor, IntakeMotor, ShooterMotor1, ShooterMotor2
    );

    // you can leave these as they are
    public static final List<String> EncoderNamesList = Arrays.asList(
            LeftOdometry, PerpendicularOdometry
    );

    public static List<String> ServoNamesList = Arrays.asList(
            TiltLeftServo, TiltRightServo, ShooterHoodServo
    );

    public static List<String> CRServoNamesList = Arrays.asList(
            LeftFront_servo, LeftBack_servo, RightFront_servo, RightBack_servo
    );

    public static List<String> DigitalNamesList = Arrays.asList(
            BreakBeam, IndexerLimit
    );

    public static List<String> RevDistanceNameList = Arrays.asList("", "");
    public static List<String> RevTouchNameList = Arrays.asList("", "");


    public static List<String> AnalogNamesList = Arrays.asList(
            LeftFront_encoder, LeftBack_encoder, RightFront_encoder, RightBack_encoder
    );

    public static List<String> RevColorNameList = Arrays.asList(
        IntakeColor
    );

}
