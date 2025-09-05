package org.firstinspires.ftc.teamcode.Hardware.Robot.Components;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.AnalogNamesList;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.CRServoNamesList;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.DigitalNamesList;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.MotorNamesList;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.RevColorNameList;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.RevDistanceNameList;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.RevTouchNameList;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.ServoNamesList;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Hardware.Robot.Localizer.IMU.SketchyIMU;
import org.firstinspires.ftc.teamcode.Hardware.Util.MotionHardware.Init;
import org.firstinspires.ftc.teamcode.Hardware.Util.SensorsEx.HubBulkRead;

import java.util.HashMap;
import java.util.Map;

public class Hardware {
    private static Hardware instance;
    public final HardwareMap hardwareMap;

    public final VoltageSensor batteryVoltageSensor;
    public final MultipleTelemetry telemetry;
    public final HubBulkRead bulk;
    public final SketchyIMU imu;



    public Map<String, Servo> servos = new HashMap<>();
    public Map<String , CRServo> CRservos = new HashMap<>();

    public Map<String, AnalogInput> analog = new HashMap<>();
    public Map<String, DigitalChannel> digital = new HashMap<>();



    public Map<String, ColorRangeSensor> color = new HashMap<>();
    public Map<String, Rev2mDistanceSensor> distance = new HashMap<>();
    public Map<String, RevTouchSensor> touch = new HashMap<>();



    public Map<String, DcMotorEx> motors = new HashMap<>();



    public static Hardware getInstance(LinearOpMode opMode) {
        if (instance == null) {
            instance = new Hardware(opMode);
        }
        return instance;
    }




    public Hardware(LinearOpMode opMode) {
        this.telemetry = new MultipleTelemetry(opMode.telemetry, FtcDashboard.getInstance().getTelemetry());
        this.bulk = new HubBulkRead(opMode.hardwareMap, LynxModule.BulkCachingMode.MANUAL);
        batteryVoltageSensor = opMode.hardwareMap.voltageSensor.iterator().next();
        this.hardwareMap = opMode.hardwareMap;
        this.imu = new SketchyIMU(opMode);

        // add all servos into a list
        for (String servoName : ServoNamesList)
            if (!servoName.isEmpty())
                servos.put(servoName, hardwareMap.get(Servo.class, servoName));

        for (String CRServoName: CRServoNamesList)
            if (!CRServoName.isEmpty())
                CRservos.put(CRServoName, hardwareMap.get(CRServo.class, CRServoName));

        // add all motors into a list
        for (String motorName : MotorNamesList)
            if (!motorName.isEmpty())
                motors.put(motorName, Init.initializeMotor(hardwareMap.get(DcMotorEx.class, motorName)));

        for (String analogName : AnalogNamesList)
            if (!analogName.isEmpty())
                analog.put(analogName, hardwareMap.get(AnalogInput.class, analogName));

        for (String digitalName : DigitalNamesList)
            if (!digitalName.isEmpty())
                digital.put(digitalName, hardwareMap.get(DigitalChannel.class, digitalName));

        for (String distanceName : RevDistanceNameList)
            if (!distanceName.isEmpty())
                distance.put(distanceName, hardwareMap.get(Rev2mDistanceSensor.class, distanceName));

        for (String colorName : RevColorNameList)
            if (!colorName.isEmpty())
                color.put(colorName, hardwareMap.get(RevColorSensorV3.class, colorName));

        for (String touchName : RevTouchNameList)
            if (!touchName.isEmpty())
                touch.put(touchName, hardwareMap.get(RevTouchSensor.class, touchName));
    }
}
