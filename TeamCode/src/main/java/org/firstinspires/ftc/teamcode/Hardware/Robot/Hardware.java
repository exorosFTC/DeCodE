package org.firstinspires.ftc.teamcode.Hardware.Robot;

import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.startPose;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Hardware.Constants.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Util.SensorsEx.HubBulkRead;
import org.firstinspires.ftc.teamcode.Pathing.Localizer.PinpointLocalizer;

public class Hardware {
    private static Hardware instance;
    public final HardwareMap hardwareMap;

    public final VoltageSensor batteryVoltageSensor;
    public final MultipleTelemetry telemetry;
    public final HubBulkRead bulk;

    public final PinpointLocalizer localizer;
    public final HuskyLens huskyLens;

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
            RightBack_servo;

    public Servo
            TiltLeftServo,
            TiltRightServo,
            ShooterHoodServo;

    public DigitalChannel
            IndexerLimit;

    public AnalogInput
            RightFront_encoder,
            RightBack_encoder,
            LeftFront_encoder,
            LeftBack_encoder;

    public RevColorSensorV3
            IntakeColor;





    public static Hardware getInstance(LinearOpMode opMode) {
        if (instance == null) {
            instance = new Hardware(opMode);
        }
        return instance;
    }




    public Hardware(LinearOpMode opMode) {
        this.telemetry = new MultipleTelemetry(opMode.telemetry, FtcDashboard.getInstance().getTelemetry());
        this.bulk = new HubBulkRead(opMode.hardwareMap, LynxModule.BulkCachingMode.MANUAL);
        this.huskyLens = opMode.hardwareMap.get(HuskyLens.class, "ExoCamera");
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

        TiltLeftServo = hardwareMap.get(Servo.class, "TiltL");
        TiltRightServo = hardwareMap.get(Servo.class, "TiltR");
        ShooterHoodServo = hardwareMap.get(Servo.class, "hood");

        IndexerLimit = hardwareMap.get(DigitalChannel.class, "indexerLim");

        IntakeColor = hardwareMap.get(RevColorSensorV3.class, "intakeColorSensor");

        RightFront_encoder = hardwareMap.get(AnalogInput.class, "RF encoder");
        RightBack_encoder = hardwareMap.get(AnalogInput.class, "RB encoder");
        LeftFront_encoder = hardwareMap.get(AnalogInput.class, "LF encoder");
        LeftBack_encoder = hardwareMap.get(AnalogInput.class, "LB encoder");


        Shooter1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Shooter1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        Shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        Shooter2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Shooter2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        Shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        IntakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        IntakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        IndexerMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        ((ServoImplEx) TiltLeftServo).setPwmRange(new PwmControl.PwmRange(500, 2500, 5000));
        ((ServoImplEx) TiltRightServo).setPwmRange(new PwmControl.PwmRange(500, 2500, 5000));
    }

    public void read(SystemBase system, SystemBase swerve) {
        bulk.clearCache(Enums.Hubs.ALL);
        batteryVoltage = batteryVoltageSensor.getVoltage();
        localizer.update();

        system.read();
        swerve.read();
    }

    public void write(SystemBase system, SystemBase swerve) {
        system.write();
        swerve.write();
    }
}
