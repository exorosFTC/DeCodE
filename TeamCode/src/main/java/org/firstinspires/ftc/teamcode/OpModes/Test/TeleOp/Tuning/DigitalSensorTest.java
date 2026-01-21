package org.firstinspires.ftc.teamcode.OpModes.Test.TeleOp.Tuning;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.CommandBase.Robot.Hardware;
import org.firstinspires.ftc.teamcode.CommandBase.Util.SensorsEx.HubBulkRead;

@TeleOp(group = "test")
public class DigitalSensorTest extends LinearOpMode {

    private GamepadEx g1;
    private Hardware hardware;

    private DigitalChannel[] sensors;
    private int index = 0;

    @Override
    public void runOpMode() {
        hardware = Hardware.getInstance(this);
        g1 = new GamepadEx(gamepad1);

        sensors = new DigitalChannel[]{
                hardware.IndexerLimit,
                hardware.LeftBreakBeam,
                hardware.RightBreakBeam
        };

        for (DigitalChannel s : sensors) {
            s.setMode(DigitalChannel.Mode.INPUT);
        }

        waitForStart();

        while (opModeIsActive()) {
            g1.readButtons();

            if (g1.wasJustPressed(GamepadKeys.Button.B)) {
                index = (index + 1) % sensors.length;
            }

            boolean state = sensors[index].getState();

            hardware.bulk.clearCache(HubBulkRead.Hubs.ALL);
            hardware.telemetry.addData("Sensor", index);
            hardware.telemetry.addData("State", state ? "HIGH" : "LOW");
            hardware.telemetry.addData("Triggered", !state);
            hardware.telemetry.update();
        }
    }
}
