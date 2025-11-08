package org.firstinspires.ftc.teamcode.OpModes.Test.TeleOp;

import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.AngularD;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.AngularP;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.POSE;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.swerveD;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.swerveP;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.SystemConstants.opModeType;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Constants.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Drivetrain.Swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Drivetrain.Mecanum.MecanumDrive;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

@Config
@TeleOp(group = "test", name = "DRIVE")
public class DriveTest extends LinearOpMode {
    private SwerveDrive swerve;
    private Hardware hardware;

    private GamepadEx g1;

    public static double p = AngularP;
    public static double d = AngularD;


    @Override
    public void runOpMode() throws InterruptedException {
        swerve = new SwerveDrive(this);
        hardware = new Hardware(this);

        opModeType = Enums.OpMode.TELE_OP;

        g1 = new GamepadEx(gamepad1);

        waitForStart();

        while (opModeIsActive()) {
            swerve.update(new Pose(
                    -g1.getLeftY(),
                    g1.getLeftX(),
                    g1.getRightX() * 0.5
            ));

            hardware.telemetry.addData("x", POSE.x);
            hardware.telemetry.addData("y", POSE.y);
            hardware.telemetry.addData("heading", POSE.heading);

            hardware.telemetry.update();
            
            g1.readButtons();
            hardware.bulk.clearCache(Enums.Hubs.ALL);
        }
    }
}
