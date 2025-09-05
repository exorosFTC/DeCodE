package org.firstinspires.ftc.teamcode.OpModes.Test.TeleOp.Tuning;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Localizer;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Localizer.RR.TwoWheelNew;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

@TeleOp(name = "RR Localizer", group = "tuning")
public class LocalizationTest extends LinearOpMode {
    private Localizer localizer;
    private Hardware hardware;
    private Drivetrain drive;

    private GamepadEx g1;

    @Override
    public void runOpMode() throws InterruptedException {
        hardware = Hardware.getInstance(this);
        localizer = new TwoWheelNew(this);
        drive = new Drivetrain(this);

        g1 = new GamepadEx(gamepad1);


        waitForStart();

        localizer.setPositionEstimate(new Pose());

        while (opModeIsActive()) {
            drive.update(new Pose(
                    g1.getLeftY(),
                    -g1.getLeftX(),
                    -g1.getRightX()
            ));

            localizer.update();
            hardware.bulk.clearCache(Enums.Hubs.ALL);

            hardware.telemetry.addData("x", Math.round(localizer.getRobotPosition().x * 100) / 100);
            hardware.telemetry.addData("y", Math.round(localizer.getRobotPosition().y * 100) / 100);

            hardware.telemetry.addData("heading (deg)", localizer.getRobotPosition().heading);
            hardware.telemetry.update();
        }

    }
}