package org.firstinspires.ftc.teamcode.OpModes.Main.TeleOp;


import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.AngularD;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.AngularP;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.POSE;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.HardwareNames.IndexerLimit;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.HardwareNames.IndexerMotor;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.HardwareNames.IntakeMotor;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.SystemConstants.telemetryAddLoopTime;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hardware.Constants.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Drivetrain.Swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Data;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Scoring.ScoringSystem;
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

    public static double p = AngularP, d = AngularD;


    @Override
    protected void Init() {
        hardware = Hardware.getInstance(this);
        g1 = new GamepadEx(gamepad1);
        g2 = new GamepadEx(gamepad2);

        swerve = new SwerveDrive(this);
        system = new ScoringSystem(this);

        new Data()
                .add(Enums.OpMode.TELE_OP)
                .setAutoOnBlue(false)
                .getLoopTime(true)
                .setUsingOpenCv(false)
                .setUsingAprilTag(false)
                .setUsingAcceleration(false)
                .setUsingExponentialInput(false)
                .setUsingFieldCentric(true);

        intakeTriggers = new TriggerManager()
                .addTrigger(() -> g2.wasJustPressed(GamepadKeys.Button.B),
                            () -> {
                            if (system.intake.on)
                                    system.intake.off();
                            else system.intake.on(); })
                .addTrigger(() -> g2.wasJustPressed(GamepadKeys.Button.A),
                            () -> system.intake.reverse())

                .addTrigger(() -> g2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER),
                            () -> system.indexer.index(1))
                .addTrigger(() -> g2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER),
                            () -> system.indexer.home());

        shooterTriggers = new TriggerManager()
                .addTrigger(() -> g2.wasJustPressed(GamepadKeys.Button.DPAD_UP),
                            () -> {
                            if (system.shooter.on)
                                system.shooter.off();
                            else system.shooter.on();
                            })
                .addTrigger(() -> g2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT),
                            () -> {
                            if (system.shooter.targetPower == 0) return;

                            swerve.setLockedX(true);
                            hardware.telemetry.clear();

                            if (system.intake.on) {
                                system.isIntakeEnabled = false;
                                system.intake.off();
                            }

                            while (!system.shooter.ready() && opModeIsActive()) {
                                if (g2.wasJustPressed(GamepadKeys.Button.X))
                                    break; // fail safe, in case of idk
                                g2.readButtons();
                                system.shooter.update();
                            }
                            system.indexer.shoot(1);
                            while (system.indexer.isBusy() && opModeIsActive()) { system.shooter.update(); }

                            while (!system.shooter.ready() && opModeIsActive()) {
                                if (g2.wasJustPressed(GamepadKeys.Button.X))
                                    break; // fail safe, in case of idk
                                g2.readButtons();
                                system.shooter.update();
                            }
                            system.indexer.shoot(1);
                            while (system.indexer.isBusy() && opModeIsActive()) { system.shooter.update(); }

                            while (!system.shooter.ready() && opModeIsActive()) {
                                if (g2.wasJustPressed(GamepadKeys.Button.X))
                                    break; // fail safe, in case of idk
                                g2.readButtons();
                                system.shooter.update();;
                            }
                            system.indexer.shoot(1);
                            while (system.indexer.isBusy() && opModeIsActive()) { system.shooter.update(); }

                            system.shooter.off();
                            system.isIntakeEnabled = true;
                            system.indexer.previousLastElement = Enums.ArtifactColor.NONE;

                            swerve.setLockedX(false);
                            swerve.setLocked(true);

                            system.indexer.home();
                            })

                .addTrigger(() -> g2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER),   // add these in both cases
                        () -> system.indexer.index(1))
                .addTrigger(() -> g2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER),
                        () -> system.indexer.home());
    }

    @Override
    protected void WhenStarted() {
        new Thread(() -> {
            while (opModeIsActive()) {
                swerve.update(new Pose(
                        -g1.getLeftY(),
                        g1.getLeftX(),
                        g1.getRightX() * 0.5
                ));

                swerve.setHeadingPID(p, 0, d);
                swerve.lockHeadingToGoal(g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1);

                g1.readButtons();
                hardware.bulk.clearCache(Enums.Hubs.ALL);
            }
        }).start();

        system.indexer.home();
    }

    @Override
    protected void Loop() {
        g1.readButtons();
        g2.readButtons();

        if (system.isIntakeEnabled)
            intakeTriggers.check();
        shooterTriggers.check();

        hardware.telemetry.addData("targetHeading", swerve.targetHeading);
        hardware.telemetry.addData("x", swerve.localizer.getRobotPosition().x);
        hardware.telemetry.addData("y", swerve.localizer.getRobotPosition().y);

        system.update();
        updateTelemetry();
    }

    public void updateTelemetry() {
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
