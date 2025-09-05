package org.firstinspires.ftc.teamcode.OpModes.Test.TeleOp.Tuning.Configuration;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.AnalogNamesList;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.ServoNamesList;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Util.SensorsEx.UltrasonicSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name = "ConfigureUltrasonic", group = "tuning")
public class UltrasonicTest extends LinearOpMode {
    private UltrasonicSensor ultra;


    private Telemetry dashboardTelemetry;
    private GamepadEx g2;

    private static int index = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        g2 = new GamepadEx(gamepad2);
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ultra = new UltrasonicSensor(this, AnalogNamesList.get(index));

        waitForStart();

        while (opModeIsActive()) {

            if (g2.wasJustPressed(GamepadKeys.Button.B)) {
                index += 1;

                if (index == AnalogNamesList.size())
                    index = 0;

                ultra = new UltrasonicSensor(this, AnalogNamesList.get(index));

            }

            dashboardTelemetry.addData("name: ", AnalogNamesList.get(index));
            dashboardTelemetry.addData("raw voltage: ", ultra.getVoltage());
            dashboardTelemetry.addData("distance (IN): ", ultra.getDistance(DistanceUnit.INCH));

            dashboardTelemetry.update();
            g2.readButtons();

        }
    }
}
