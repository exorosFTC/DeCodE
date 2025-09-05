package org.firstinspires.ftc.teamcode.OpModes.Test.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Drivetrain;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

@TeleOp(group = "test", name = "DRIVE")
public class DriveTest extends LinearOpMode {
    private Drivetrain drive;
    private Hardware hardware;

    private boolean testingLocalizer = false;

    private GamepadEx g1;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new Drivetrain(this);
        hardware = new Hardware(this);

        g1 = new GamepadEx(gamepad1);

        waitForStart();

        while (opModeIsActive()) {
            drive.update(new Pose(
                    g1.getLeftY(),
                    -g1.getLeftX(),
                    -g1.getRightX() * 0.7
            ));

            g1.readButtons();
        }
    }
}
