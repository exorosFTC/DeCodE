package org.firstinspires.ftc.teamcode.CommandBase.Robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.CommandBase.Util.SensorsEx.HubBulkRead;
import org.firstinspires.ftc.teamcode.CommandBase.Util.SensorsEx.LimelightEx;
import org.firstinspires.ftc.teamcode.CustomPathing.Localizer.PinpointLocalizer;

public class Hardware {
    private static Hardware instance;
    public final HardwareMap hardwareMap;

    private double startLoopTime;

    public final VoltageSensor batteryVoltageSensor;
    public final MultipleTelemetry telemetry;
    public final HubBulkRead bulk;

    public final PinpointLocalizer localizer;
    public final LimelightEx limelight;

    public double batteryVoltage = 14;

    public final DcMotorEx
            LeftFront,
            LeftBack,
            RightBack,
            RightFront,
            IntakeMotor,
            IndexerMotor,
            Shooter1,
            Shooter2;

    public CRServo
            LeftFront_servo,
            LeftBack_servo,
            RightFront_servo,
            RightBack_servo,

            LeftFront_lift,
            LeftBack_lift,
            RightFront_lift,
            RightBack_lift;

    public Servo
            ShooterHoodServo,
            LiftRetainerServo;

    public DigitalChannel
            IndexerLimit,
            LeftBreakBeam,
            RightBreakBeam;

    public AnalogInput
            RightFront_encoder,
            RightBack_encoder,
            LeftFront_encoder,
            LeftBack_encoder;

    public RevColorSensorV3
            IntakeColor1,
            IntakeColor2,
            IntakeColor3;



    public static Hardware getInstance(LinearOpMode opMode) {
        if (instance == null) {
            instance = new Hardware(opMode);
        }
        return instance;
    }

    public Hardware(LinearOpMode opMode) {
        this.telemetry = new MultipleTelemetry(opMode.telemetry, FtcDashboard.getInstance().getTelemetry());
        this.bulk = new HubBulkRead(opMode.hardwareMap, LynxModule.BulkCachingMode.MANUAL);
        this.limelight = new LimelightEx("limelight", opMode);
        this.localizer = new PinpointLocalizer(opMode.hardwareMap);

        batteryVoltageSensor = opMode.hardwareMap.voltageSensor.iterator().next();

        this.hardwareMap = opMode.hardwareMap;


        LeftFront = hardwareMap.get(DcMotorEx.class, "LF");
        LeftBack = hardwareMap.get(DcMotorEx.class, "LB");
        RightFront = hardwareMap.get(DcMotorEx.class, "RF");
        RightBack = hardwareMap.get(DcMotorEx.class, "RB");

        IntakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        IndexerMotor = hardwareMap.get(DcMotorEx.class, "indexer");
        Shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        Shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");



        LeftFront_servo = hardwareMap.get(CRServo.class, "LF servo");
        LeftBack_servo = hardwareMap.get(CRServo.class, "LB servo");
        RightFront_servo = hardwareMap.get(CRServo.class, "RF servo");
        RightBack_servo = hardwareMap.get(CRServo.class, "RB servo");

        LeftFront_lift = hardwareMap.get(CRServo.class, "LF lift");
        LeftBack_lift = hardwareMap.get(CRServo.class, "LB lift");
        RightFront_lift = hardwareMap.get(CRServo.class, "RF lift");
        RightBack_lift = hardwareMap.get(CRServo.class, "RB lift");

        ShooterHoodServo = hardwareMap.get(Servo.class, "hood");
        LiftRetainerServo = hardwareMap.get(Servo.class, "liftRetainer");



        IndexerLimit = hardwareMap.get(DigitalChannel.class, "indexerLim");
        LeftBreakBeam = hardwareMap.get(DigitalChannel.class, "leftBreakBeam");
        RightBreakBeam = hardwareMap.get(DigitalChannel.class, "rightBreakBeam");

        IntakeColor1 = hardwareMap.get(RevColorSensorV3.class, "color1");
        IntakeColor2 = hardwareMap.get(RevColorSensorV3.class, "color2");
        IntakeColor3 = hardwareMap.get(RevColorSensorV3.class, "color3");

        RightFront_encoder = hardwareMap.get(AnalogInput.class, "RF encoder");
        RightBack_encoder = hardwareMap.get(AnalogInput.class, "RB encoder");
        LeftFront_encoder = hardwareMap.get(AnalogInput.class, "LF encoder");
        LeftBack_encoder = hardwareMap.get(AnalogInput.class, "LB encoder");



        Shooter1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Shooter1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        Shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Shooter2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Shooter2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        Shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IntakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        IntakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }



    public void readBattery() { batteryVoltage = batteryVoltageSensor.getVoltage(); }

    public void updateTelemetry() {

        double endLoopTime = System.nanoTime();

        telemetry.addData(
                "Loop Time:",
                String.format("%.4f Hz", 1_000_000_000.0 / (endLoopTime - startLoopTime))
        );

        startLoopTime = endLoopTime;
        telemetry.update();
    }
}