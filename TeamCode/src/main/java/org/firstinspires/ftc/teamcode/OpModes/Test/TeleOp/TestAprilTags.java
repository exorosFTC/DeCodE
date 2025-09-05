package org.firstinspires.ftc.teamcode.OpModes.Test.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.OpenCV.AprilTagCamera;

@TeleOp(name = "TestA-Tags")
public class TestAprilTags extends LinearOpMode {
    private AprilTagCamera aprilTagCamera;
    private Telemetry dashboardTelemetry;

    @Override
    public void runOpMode() throws InterruptedException {
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        aprilTagCamera = new AprilTagCamera(this, dashboardTelemetry);

        while (!isStarted() && !isStopRequested()) { dashboardTelemetry.addData("Distance to closest april-tag: ", aprilTagCamera.getDetection()); dashboardTelemetry.update(); }
    }
}
