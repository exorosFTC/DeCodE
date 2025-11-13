package org.firstinspires.ftc.teamcode.OpModes.Main.TeleOp;


import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.AngularD;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.AngularP;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.startPose;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.SystemConstants.telemetryAddLoopTime;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Constants.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Data;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Scoring.ScoringSystem;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Scoring.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Hardware.Util.TriggerManager;
import org.firstinspires.ftc.teamcode.OpModes.ExoMode;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

@Config
@TeleOp(name = "😈🔥", group = "main")
public class CrazyTeleOp extends ExoMode {
    private double startLoopTime;

    private Hardware hardware;
    private SwerveDrive swerve;
    private ScoringSystem system;

    private GamepadEx g1, g2;
    private TriggerManager intakeTriggers, shooterTriggers;

    public static double swerveP = AngularP, swerveD = AngularD;
    public static double shooterP = Shooter.kP, shooterD = Shooter.kD;

    public static double c2_angle_adjust = 0,
            c_angle_close = 0,
            c_angle_far = 0,
            c_power = 0;

    @Override
    protected void Init() {
        hardware = Hardware.getInstance(this);
        g1 = new GamepadEx(gamepad1);
        g2 = new GamepadEx(gamepad2);

        swerve = new SwerveDrive(this);
        system = new ScoringSystem(this);

        c2_angle_adjust = Shooter.c2_angle_adjust;
        c_angle_close = Shooter.c_angle_close;
        c_angle_far = Shooter.c_angle_far;
        c_power = Shooter.c_power;

        new Data()
                .add(Enums.OpMode.TELE_OP)
                .setAutoOnBlue(false)
                .getLoopTime(true)
                .setUsingOpenCv(false)
                .setUsingAprilTag(false)
                .setUsingFieldCentric(true);

        intakeTriggers = new TriggerManager()
                .addTrigger(() -> g2.wasJustPressed(GamepadKeys.Button.B),
                            () -> {
                            if (system.intake.on) {
                                system.intake.off();
                                system.indexer.off();
                            }
                            else {
                                system.intake.on();
                                system.indexer.on();
                            }})
                .addTrigger(() -> g2.wasJustPressed(GamepadKeys.Button.A),
                            () -> system.intake.reverse());

        shooterTriggers = new TriggerManager()
                .addTrigger(() -> g2.wasJustPressed(GamepadKeys.Button.DPAD_UP),
                            () -> {
                            if (system.shooter.on)
                                system.shooter.off();
                            else system.shooter.on();
                            })
                .addTrigger(() -> g2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT),
                            () -> system.shootSequence(swerve))
                .addTrigger(() -> g2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER),   // add these in both cases
                        () -> {
                            system.indexer.index(1);
                            if (!system.intake.on) {
                                while (system.indexer.isBusy() && opModeIsActive()) { system.read(); system.write();}
                                system.indexer.off();
                            }
                        })
                .addTrigger(() -> g2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER),
                        () -> system.indexer.home())
                .addTrigger(() -> g2.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON),
                        () -> system.indexer.sideswipe(3, true))
                .addTrigger(() -> g1.wasJustPressed(GamepadKeys.Button.X),              // tilting for driver 1
                        () -> {
                            system.tilt.on();
                            try { Thread.sleep(1500); } catch (InterruptedException e) {}
                            system.tilt.off();
                        });

        try { Thread.sleep(150); } catch (InterruptedException e) {}
        hardware.localizer.setPositionEstimate(startPose);
        try { Thread.sleep(150); } catch (InterruptedException e) {}

        hardware.telemetry.addLine("INIT READY 😈😈😈");
        hardware.telemetry.update();
    }

    @Override
    protected void WhenStarted() {
        hardware.telemetry.clearAll();
        system.indexer.home();
    }

    @Override
    protected void Loop() {
        hardware.read(system, swerve);
        g1.readButtons();
        g2.readButtons();

        Shooter.c2_angle_adjust = c2_angle_adjust;
        Shooter.c_angle_close = c_angle_close;
        Shooter.c_angle_far = c_angle_far;
        Shooter.c_power = c_power;

        swerve.update(new Pose(
                -g1.getLeftY(),
                g1.getLeftX(),
                g1.getRightX() * 0.2)
        );
        system.update();

        if (system.isIntakeEnabled)
            intakeTriggers.check();
        shooterTriggers.check();

        swerve.setHeadingPID(swerveP, 0, swerveD);
        system.shooter.setPID(shooterP, 0, shooterD);
        swerve.lockHeadingToGoal(g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1);

        hardware.write(system, swerve);
        updateTelemetry();
    }



    private void updateTelemetry() {
        if (telemetryAddLoopTime) {
            double endLoopTime = System.nanoTime();

            hardware.telemetry.addData(
                    "Loop Time:",
                    String.format("%.4f Hz", 1_000_000_000.0 / (endLoopTime - startLoopTime))
            );

            startLoopTime = endLoopTime;
        }

        hardware.telemetry.update();
    }
}
