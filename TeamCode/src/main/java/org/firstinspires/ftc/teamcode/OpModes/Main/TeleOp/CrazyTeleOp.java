package org.firstinspires.ftc.teamcode.OpModes.Main.TeleOp;


import static org.firstinspires.ftc.teamcode.Hardware.Constants.HardwareNames.IndexerLimit;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.SystemConstants.telemetryAddLoopTime;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Constants.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Drivetrain.Swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Data;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Scoring.ScoringSystem;
import org.firstinspires.ftc.teamcode.Hardware.Util.TriggerManager;
import org.firstinspires.ftc.teamcode.OpModes.ExoMode;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

@TeleOp(name = "😈🔥", group = "main")
public class CrazyTeleOp extends ExoMode {
    private double startLoopTime;

    private Hardware hardware;
    private SwerveDrive swerve;
    private ScoringSystem system;

    private GamepadEx g1, g2;
    private TriggerManager intakeTriggers, shooterTriggers;



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
                            () -> {})
                .addTrigger(() -> g2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN),
                            () -> {})
                .addTrigger(() -> g2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER),
                            () -> {});
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
        else if (system.isShooterEnabled)
            shooterTriggers.check();


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
