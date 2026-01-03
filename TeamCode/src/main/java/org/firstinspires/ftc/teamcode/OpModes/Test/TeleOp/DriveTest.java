package org.firstinspires.ftc.teamcode.OpModes.Test.TeleOp;

import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.TeleOpAngularD;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.TeleOpAngularP;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.POSE;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.SystemConstants.opModeType;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.CommandBase.Constants.Enums;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

@Config
@TeleOp(group = "test", name = "DRIVE")
public class DriveTest extends LinearOpMode {
    private SwerveDrive swerve;
    private Hardware hardware;

    private GamepadEx g1;

    public static double p = TeleOpAngularP;
    public static double d = TeleOpAngularD;


    @Override
    public void runOpMode() {
        swerve = new SwerveDrive(this);
        hardware = new Hardware(this);

        opModeType = Enums.OpMode.TELE_OP;

        g1 = new GamepadEx(gamepad1);

        waitForStart();

        while (opModeIsActive()) {
            g1.readButtons();
            swerve.read();

            hardware.localizer.update();
            swerve.update(new Pose(
                    g1.getLeftY(),
                    -g1.getLeftX(),
                    -g1.getRightX() * 0.1
            ));

            hardware.telemetry.addData("x", POSE.x);
            hardware.telemetry.addData("y", POSE.y);
            hardware.telemetry.addData("heading", POSE.heading);

            hardware.telemetry.addData("LF draw:", hardware.LeftFront.getCurrent(CurrentUnit.AMPS));
            hardware.telemetry.addData("LB draw:", hardware.LeftBack.getCurrent(CurrentUnit.AMPS));
            hardware.telemetry.addData("RF draw:", hardware.RightFront.getCurrent(CurrentUnit.AMPS));
            hardware.telemetry.addData("RB draw:", hardware.RightBack.getCurrent(CurrentUnit.AMPS));


            hardware.telemetry.update();
            swerve.write();

            hardware.bulk.clearCache(Enums.Hubs.ALL);
        }
    }
}
