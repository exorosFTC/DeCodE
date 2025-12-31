package org.firstinspires.ftc.teamcode.OpModes.Main.TeleOp;


import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AngularD;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AngularP;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.startPose;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.SystemConstants.telemetryAddLoopTime;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandBase.Constants.Enums;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Hardware;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.SystemData;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.ScoringSystem;
import org.firstinspires.ftc.teamcode.CommandBase.Util.TriggerManager;
import org.firstinspires.ftc.teamcode.OpModes.ExoMode;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

import java.util.concurrent.atomic.AtomicBoolean;

@Config
@TeleOp(name = "🍪", group = "main")
public class SoloTeleOp extends ExoMode {
    private double startLoopTime;

    private Hardware hardware;
    private SwerveDrive swerve;
    private ScoringSystem system;

    private GamepadEx g1;
    private final InputBus in = new InputBus();
    private TriggerManager intakeTriggers, shooterTriggers, swerveTriggers;

    private Thread swerveThread, gamepadThread;

    public static double swerveP = AngularP, swerveD = AngularD;



    @Override
    protected void Init() {
        // init hardware
        hardware = Hardware.getInstance(this);
        g1 = new GamepadEx(gamepad1);

        swerve = new SwerveDrive(this);
        system = new ScoringSystem(this);

        new SystemData()
                .add(Enums.OpMode.TELE_OP)
                .getLoopTime(true);


        // create gamepad triggers
        intakeTriggers = new TriggerManager()
                .addTrigger(() -> in.evToggleIntake.getAndSet(false), () -> {
                            if (system.intake.on && !system.intake.reversed) {
                                system.intake.off();
                                system.indexer.off();
                            } else {
                                system.intake.on();
                                system.indexer.on();
                            }}) // intake
                .addTrigger(() -> in.evReverseIntake.getAndSet(false), () -> {
                            if (system.intake.on && system.intake.reversed) {
                                system.intake.off();
                                system.indexer.off();
                            } else {
                                system.intake.reverse();
                                system.indexer.on();
                            }
                        }); // reverse intake

        shooterTriggers = new TriggerManager()
                .addTrigger(() -> in.evShootSorted.getAndSet(false), () -> {
                            while (!system.shooter.ready()) { system.update(); }

                            system.indexer.setRapidFire(false);
                            system.shootSequence();
                        })               // sorted shoot
                .addTrigger(() -> in.evShootUnsorted.getAndSet(false), () -> {
                            while (!system.shooter.ready()) { system.update(); }

                            system.indexer.setRapidFire(true);
                            system.shootSequence();
                        })                // unsorted shoot
                .addTrigger(() -> in.evHomeIndexer.getAndSet(false), () -> system.indexer.home()); // emergency homing


        // set the right start position
        try { Thread.sleep(150); } catch (InterruptedException e) {}
        hardware.localizer.setPositionEstimate(startPose);
        try { Thread.sleep(150); } catch (InterruptedException e) {}


        // initialize threads
        swerveThread = new Thread(() -> {
            while (opModeIsActive()) {
                hardware.read(system, swerve);

                swerve.update(new Pose(
                        swerve.xLim.calculate(in.ly),
                        swerve.yLim.calculate(-in.lx),
                        swerve.headLim.calculate(-in.rx * 0.05))
                );

                swerve.lockHeadingToGoal(in.lockToGoal);
                if (in.evLockX.getAndSet(false)) swerve.setLockedX(true);

                updateTelemetry();
                hardware.write(system, swerve);
            }
        }, "SwerveThread");
        gamepadThread = new Thread(() -> {
            while (opModeIsActive()) {
                g1.readButtons();

                // continuous snapshot
                in.ly = g1.getLeftY();
                in.lx = g1.getLeftX();
                in.rx = g1.getRightX();
                in.lt = g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
                in.rt = g1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
                in.lockToGoal = in.lt > 0.1;
                in.rbHeld = g1.isDown(GamepadKeys.Button.RIGHT_BUMPER);

                // edges -> events (one-shot)
                if (g1.wasJustPressed(GamepadKeys.Button.B)) in.evToggleIntake.set(true);
                if (g1.wasJustPressed(GamepadKeys.Button.A)) in.evReverseIntake.set(true);
                if (g1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) in.evShootSorted.set(true);
                if (g1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) in.evShootUnsorted.set(true);
                if (g1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) in.evHomeIndexer.set(true);
                if (g1.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) in.evLockX.set(true);

                // tiny yield to avoid maxing CPU
                Thread.yield();
            } }, "GamepadThread");


        hardware.telemetry.addLine("INIT READY 😈😈😈");
        hardware.telemetry.update();
    }

    @Override
    protected void WhenStarted() {
        hardware.telemetry.clearAll();

        gamepadThread.start();
        swerveThread.start();

        system.indexer.home();
    }

    @Override
    protected void Loop() {
        if (system.isIntakeEnabled)
            intakeTriggers.check();
        shooterTriggers.check();

        if (in.rbHeld && !system.shooter.on) {
            system.shooter.on();
            system.indexer.microAdjust();
        } else if (!in.rbHeld && system.shooter.on) system.shooter.off();

        system.update();

        try { Thread.sleep(5); } catch (InterruptedException e) {}
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



    static class InputBus {
        // continuous
        volatile double lx, ly, rx;
        volatile double lt, rt;
        volatile boolean rbHeld;
        volatile boolean lockToGoal;

        // one-shot events
        final AtomicBoolean evToggleIntake   = new AtomicBoolean(false);
        final AtomicBoolean evReverseIntake  = new AtomicBoolean(false);
        final AtomicBoolean evShootSorted    = new AtomicBoolean(false);
        final AtomicBoolean evShootUnsorted  = new AtomicBoolean(false);
        final AtomicBoolean evHomeIndexer    = new AtomicBoolean(false);
        final AtomicBoolean evLockX          = new AtomicBoolean(false);
    }
}
