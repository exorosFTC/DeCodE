package org.firstinspires.ftc.teamcode.OpModes.Test.TeleOp.Tuning;

import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.startPose;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Constants.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Data;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Scoring.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Pathing.Localizer.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

@Config
@TeleOp(name = "Shooter", group = "main")
public class ShooterTunning extends LinearOpMode {
    private Hardware hardware;
    private SwerveDrive swerve;
    private Shooter shooter;

    private GamepadEx g1;
    private PinpointLocalizer localizer;


    public static double c2_angle_adjust = 0,
                         c_angle_close = 0,
                         c_angle_far = 0,
                         c_power = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        hardware = Hardware.getInstance(this);
        swerve = new SwerveDrive(this);
        shooter = new Shooter(this);

        g1 = new GamepadEx(gamepad1);

        localizer = new PinpointLocalizer(hardwareMap);

        try { Thread.sleep(200); } catch (InterruptedException e) {}
        localizer.setPositionEstimate(startPose);

        new Data()
                .add(Enums.OpMode.TELE_OP)
                .setAutoOnBlue(false)
                .getLoopTime(true)
                .setUsingOpenCv(false)
                .setUsingAprilTag(false)
                .setUsingFieldCentric(true);

        c2_angle_adjust = Shooter.c2_angle_adjust;
        c_angle_close = Shooter.c_angle_close;
        c_angle_far = Shooter.c_angle_far;
        c_power = Shooter.c_power;

        waitForStart();

        new Thread(() -> {
            while (opModeIsActive()) {

                swerve.update(new Pose(
                        -g1.getLeftY(),
                        g1.getLeftX(),
                        g1.getRightX())
                );

                swerve.lockHeadingToGoal(g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1);

                if (g1.wasJustPressed(GamepadKeys.Button.B))
                    shooter.on();
                else if (g1.wasJustPressed(GamepadKeys.Button.A))
                    shooter.off();

                g1.readButtons();
                hardware.bulk.clearCache(Enums.Hubs.ALL);
            }
        }).start();

        while (opModeIsActive()) {

            Shooter.c2_angle_adjust = c2_angle_adjust;
            Shooter.c_angle_close = c_angle_close;
            Shooter.c_angle_far = c_angle_far;
            Shooter.c_power = c_power;

            localizer.update();
            shooter.update();

            hardware.telemetry.addData("distance", shooter.distance);
            hardware.telemetry.addData("velocity target", shooter.TARGET);
            hardware.telemetry.addData("velocity", shooter.wheelVelocity);
            hardware.telemetry.addData("ready", shooter.ready());

            hardware.bulk.clearCache(Enums.Hubs.ALL);
            hardware.telemetry.update();
        }
    }
}
