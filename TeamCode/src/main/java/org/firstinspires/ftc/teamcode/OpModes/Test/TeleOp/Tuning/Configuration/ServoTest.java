package org.firstinspires.ftc.teamcode.OpModes.Test.TeleOp.Tuning.Configuration;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.ServoNamesList;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.List;

@Config
@TeleOp(name = "ConfigureServo", group = "tuning")
public class ServoTest extends LinearOpMode {
    private Servo servo;


    private Telemetry dashboardTelemetry;
    private GamepadEx g2;

    private static int index = 0;

    private double position = 0;
    private double increment = 0.05;
    private final double incrementSmall = 0.01;

    @Override
    public void runOpMode() throws InterruptedException {
        g2 = new GamepadEx(gamepad2);
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        servo = hardwareMap.get(Servo.class, ServoNamesList.get(index));

        waitForStart();

        while (opModeIsActive()) {

            if (g2.wasJustPressed(GamepadKeys.Button.B)) {
                index += 1;

                if (index == ServoNamesList.size())
                    index = 0;

                servo = hardwareMap.get(Servo.class, ServoNamesList.get(index));

            }

            if (g2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                position -= increment;

                position = Range.clip(position, 0, 1);
                servo.setPosition(position);
            }
            else if (g2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                position += increment;

                position = Range.clip(position, 0, 1);
                servo.setPosition(position);
            }

            if (g2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                if (increment == incrementSmall)
                    increment = 0.05;
                else increment = incrementSmall;
            }


            dashboardTelemetry.addData("position: ", position);
            dashboardTelemetry.addData("servo: ", ServoNamesList.get(index));

            dashboardTelemetry.update();
            g2.readButtons();

        }

    }
}
