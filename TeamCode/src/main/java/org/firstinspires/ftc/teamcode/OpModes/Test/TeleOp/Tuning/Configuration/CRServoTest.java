package org.firstinspires.ftc.teamcode.OpModes.Test.TeleOp.Tuning.Configuration;

import static org.firstinspires.ftc.teamcode.Hardware.Constants.HardwareNames.AnalogNamesList;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.HardwareNames.CRServoNamesList;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.Util.SensorsEx.AbsoluteAnalogEncoder;

@TeleOp(name = "ConfigureCRServo", group = "tuning")
public class CRServoTest extends LinearOpMode {
    private CRServo servo;
    private AbsoluteAnalogEncoder encoder;

    private Telemetry dashboardTelemetry;
    private GamepadEx g2;


    private static int index = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        g2 = new GamepadEx(gamepad2);
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        servo = hardwareMap.get(CRServo.class, CRServoNamesList.get(index));
        encoder = new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, AnalogNamesList.get(index)));

        waitForStart();

        while (opModeIsActive()) {

            if (g2.wasJustPressed(GamepadKeys.Button.B)) {
                index += 1;

                if (index == CRServoNamesList.size())
                    index = 0;

                servo = hardwareMap.get(CRServo.class, CRServoNamesList.get(index));
                encoder = new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, AnalogNamesList.get(index)));
            }

            servo.setPower(g2.getLeftY());


            dashboardTelemetry.addData("servo: ", CRServoNamesList.get(index));
            dashboardTelemetry.addData("position: ", encoder.getCurrentPosition(AngleUnit.RADIANS));


            dashboardTelemetry.update();
            g2.readButtons();
        }
    }
}
