package org.firstinspires.ftc.teamcode.OpModes.Test.TeleOp.Tuning;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Hardware;

@TeleOp(group = "test")
public class ColorSensorTest extends LinearOpMode {

    private GamepadEx g1;
    private Hardware hardware;

    private NormalizedColorSensor[] sensors;
    private DistanceSensor[] dist;

    private int index = 0;

    @Override
    public void runOpMode() {
        hardware = Hardware.getInstance(this);
        g1 = new GamepadEx(gamepad1);

        sensors = new NormalizedColorSensor[]{
                hardware.IntakeColor1,
                hardware.IntakeColor2,
                hardware.IntakeColor3
        };

        dist = new DistanceSensor[]{
                (DistanceSensor) hardware.IntakeColor1,
                (DistanceSensor) hardware.IntakeColor2,
                (DistanceSensor) hardware.IntakeColor3
        };

        waitForStart();

        while (opModeIsActive()) {
            g1.readButtons();

            if (g1.wasJustPressed(GamepadKeys.Button.B)) {
                index = (index + 1) % sensors.length;
            }

            NormalizedRGBA c = sensors[index].getNormalizedColors();
            double d = dist[index].getDistance(DistanceUnit.MM);

            hardware.telemetry.addData("Sensor", index);
            hardware.telemetry.addData("Dist (mm)", "%.1f", d);
            hardware.telemetry.addData("R", "%.3f", c.red);
            hardware.telemetry.addData("G", "%.3f", c.green);
            hardware.telemetry.addData("B", "%.3f", c.blue);
            hardware.telemetry.addData("A", "%.3f", c.alpha);
            hardware.telemetry.addData("Dominant", dominant(c.red, c.green, c.blue));
            hardware.telemetry.update();
        }
    }

    private String dominant(double r, double g, double b) {
        if (r > g && r > b) return "RED";
        if (g > r && g > b) return "GREEN";
        if (b > r && b > g) return "BLUE";
        return "MIX";
    }
}
