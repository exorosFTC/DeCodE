package org.firstinspires.ftc.teamcode.OpModes.Test.TeleOp.Tuning.Configuration;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.RevColorNameList;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;

import java.util.Iterator;
import java.util.Map;

@TeleOp(name = "ConfigureSensor", group = "tuning")
public class ColorSensorTest extends LinearOpMode {
    private ColorRangeSensor color;
    private Hardware hardware;

    Iterator<Map.Entry<String, ColorRangeSensor>> iterator;
    Map.Entry<String, ColorRangeSensor> current;

    private GamepadEx g2;

    @Override
    public void runOpMode() throws InterruptedException {
        hardware = Hardware.getInstance(this);
        g2 = new GamepadEx(gamepad2);

        iterator = hardware.color.entrySet().iterator();
        current = iterator.next();
        color = current.getValue();


        waitForStart();



        while (opModeIsActive()) {
            if (g2.wasJustPressed(GamepadKeys.Button.B)) {

                if (!iterator.hasNext())
                    iterator = hardware.color.entrySet().iterator();

                current = iterator.next();
                color = current.getValue();
            }

            hardware.telemetry.addData("Red: ", color.red());
            hardware.telemetry.addData("Green: ", color.green());
            hardware.telemetry.addData("Blue: ", color.blue());
            hardware.telemetry.addData("Distance: ", color.getDistance(DistanceUnit.MM));

            g2.readButtons();
            hardware.bulk.clearCache(Enums.Hubs.ALL);
            hardware.telemetry.update();
        }

    }
}
