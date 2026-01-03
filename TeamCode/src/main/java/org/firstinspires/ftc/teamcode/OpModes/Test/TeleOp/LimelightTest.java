package org.firstinspires.ftc.teamcode.OpModes.Test.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandBase.Util.SensorsEx.LimelightEx;

@TeleOp(name = "RandomizationTest", group = "test")
public class LimelightTest extends LinearOpMode {
    private LimelightEx limelight;
    private MultipleTelemetry dashboardTelemetry;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = new LimelightEx("limelight", this.hardwareMap);
        dashboardTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        limelight.start();

        while (opModeIsActive()) {
            limelight.read();

            dashboardTelemetry.addData("random:", limelight.getRandomization());
            dashboardTelemetry.update();
        }
    }
}
