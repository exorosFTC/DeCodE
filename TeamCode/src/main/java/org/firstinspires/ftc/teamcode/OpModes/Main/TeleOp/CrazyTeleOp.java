package org.firstinspires.ftc.teamcode.OpModes.Main.TeleOp;

import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.POSE;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.goalPosition;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.goalPositionBlue;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.goalPositionRed;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.startPose;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.SystemConstants.autoOnBlue;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandBase.Constants.SystemConstants;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Hardware;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.SystemData;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.ScoringSystem;
import org.firstinspires.ftc.teamcode.CommandBase.Util.SensorsEx.HubBulkRead;
import org.firstinspires.ftc.teamcode.CommandBase.Util.TriggerManager;
import org.firstinspires.ftc.teamcode.CommandBase.Util.InputBus;
import org.firstinspires.ftc.teamcode.OpModes.ExoMode;
import org.firstinspires.ftc.teamcode.CustomPathing.Math.Geometry.Pose;

@Config
@TeleOp(name = "😈🔥", group = "main")
public class CrazyTeleOp extends ExoMode {

    private Hardware hardware;
    private SwerveDrive swerve;
    private ScoringSystem system;
    private Lift lift;

    private GamepadEx g1, g2;
    private final InputBus in = new InputBus();

    private TriggerManager intakeTriggers, shooterTriggers;
    private Thread swerveThread, gamepadThread;

    @Override
    protected void Init() {
        // init hardware
        hardware = Hardware.getInstance(this);
        g1 = new GamepadEx(gamepad1);
        g2 = new GamepadEx(gamepad2);

        swerve = new SwerveDrive(this);
        system = new ScoringSystem(this);
        lift = new Lift(this);

        new SystemData()
                .add(SystemConstants.OpMode.TELE_OP);

        // create gamepad triggers
        intakeTriggers = new TriggerManager()
                .addTrigger(() -> in.evToggleIntake.getAndSet(false), () -> {
                    if (system.intake.on && !system.intake.reversed) system.intake.off();
                    else system.intake.on();
                })   // intake
                .addTrigger(() -> in.evReverseIntake.getAndSet(false), () -> {
                    if (system.intake.on && system.intake.reversed) system.intake.off();
                    else system.intake.reverse();
                }); // reverse intake

        shooterTriggers = new TriggerManager()
                .addTrigger(() -> in.evSort.getAndSet(false), () -> system.indexer.indexPattern())                // sort
                .addTrigger(() -> in.evShoot.getAndSet(false) && in.spinupShooter, () -> system.shootSequence())  // shoot
                .addTrigger(() -> in.evHomeIndexer.getAndSet(false), () -> system.indexer.home());                // emergency homing


        // set the right start position
        try { Thread.sleep(150); } catch (InterruptedException e) {}
        hardware.localizer.setPositionEstimate(startPose);
        try { Thread.sleep(150); } catch (InterruptedException e) {}


        // initialize threads
        swerveThread = new Thread(() -> {
            while (opModeIsActive()) {
                if (!swerve.on) { lift.read(); lift.write(); continue; }

                swerve.read();
                swerve.update(new Pose(
                        in.ly,
                        -in.lx,
                        -in.rx * 0.85)
                );
                swerve.write();

                swerve.lockHeadingToGoal(in.lockToGoal);
                if (in.evLockX.getAndSet(false)) swerve.setLockedX(true);
                if (in.evResetHeading.getAndSet(false)) hardware.localizer.setPositionEstimate(new Pose(POSE.x, POSE.y, 0));
                if (in.evResetPosition.getAndSet(false)) {
                    if (autoOnBlue) hardware.localizer.setPositionEstimate(new Pose(-160, -160, 0));
                    else hardware.localizer.setPositionEstimate(new Pose(-160, 160, 0));
                }
                if (in.evStartLift.getAndSet(false)) {
                    swerve.disable();
                    system.indexer.off();
                    system.intake.off();
                    system.shooter.off();

                    lift.on();
                }
                if (in.evSetBlue.getAndSet(false)) { goalPosition = goalPositionBlue; autoOnBlue = true; };
                if (in.evSetRed.getAndSet(false)) { goalPosition = goalPositionRed; autoOnBlue = false; };

                hardware.updateTelemetry();
            }
        }, "SwerveThread");
        gamepadThread = new Thread(() -> {
            while (opModeIsActive()) {
                hardware.bulk.clearCache(HubBulkRead.Hubs.ALL);
                hardware.read(system);

                g1.readButtons();
                g2.readButtons();

                // continuous snapshot
                in.ly2 = g2.getLeftY();
                in.ly = g1.getLeftY();
                in.lx = g1.getLeftX();
                in.rx = g1.getRightX();
                in.lt = g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
                in.rt = g2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
                in.lockToGoal = in.lt > 0.1;
                in.spinupShooter = in.rt > 0.1;

                // edges -> events (one-shot)
                if (g2.wasJustPressed(GamepadKeys.Button.B)) in.evToggleIntake.set(true);
                if (g2.wasJustPressed(GamepadKeys.Button.A)) in.evReverseIntake.set(true);
                if (g2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) in.evSort.set(true);
                if (g2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) in.evShoot.set(true);
                if (g2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) in.evHomeIndexer.set(true);

                if (g1.isDown(GamepadKeys.Button.X) && g1.isDown(GamepadKeys.Button.DPAD_RIGHT)) in.evStartLift.set(true);
                if (g1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) in.evResetHeading.set(true);
                if (g1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) in.evResetPosition.set(true);
                if (g1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) in.evSetRed.set(true);
                if (g1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) in.evSetBlue.set(true);

                if (g1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1) in.evLockX.set(true);

                system.indexer.manual(in.ly2, 0.2);
                system.shooter.update();
                system.write();

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

        lift.init();
        system.indexer.home();
    }

    @Override
    protected void Loop() {
        intakeTriggers.check();
        shooterTriggers.check();

        if (in.spinupShooter && !system.shooter.on) {
            system.shooter.on();
            system.indexer.microAdjust(false);
        } else if (!in.spinupShooter && system.shooter.on) {
            system.shooter.off();
            system.indexer.microAdjust(true);
        }

        system.updateIntake();
        Thread.yield();
    }
}
