package org.firstinspires.ftc.teamcode.OpModes.Test.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Util.SensorsEx.UltrasonicSensor;

@TeleOp(group = "test")
public class UltrasonicTest extends LinearOpMode {
    UltrasonicSensor sensor;

    @Override
    public void runOpMode() throws InterruptedException {
        sensor = new UltrasonicSensor(this, "sensor");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Distance (MM): ", sensor.getDistance(DistanceUnit.MM));
            telemetry.addData("Distance (Inch): ", sensor.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}
