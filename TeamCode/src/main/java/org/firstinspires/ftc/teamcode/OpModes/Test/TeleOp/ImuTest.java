package org.firstinspires.ftc.teamcode.OpModes.Test.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Localizer.IMU.SketchyIMU;

@TeleOp(group = "test", name = "IMU")
public class ImuTest extends LinearOpMode {
    private SketchyIMU imu;
    private Telemetry dashboardTelemetry;

    @Override
    public void runOpMode() throws InterruptedException {
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        imu = new SketchyIMU(this);

        waitForStart();

        imu.reset();
        while (opModeIsActive()) {
            dashboardTelemetry.addData("Angle DEG: ", imu.getAngle(AngleUnit.DEGREES));
            dashboardTelemetry.update();
        }
    }
}
