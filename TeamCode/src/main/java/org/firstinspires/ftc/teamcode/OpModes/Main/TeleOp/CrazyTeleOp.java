package org.firstinspires.ftc.teamcode.OpModes.Main.TeleOp;

import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.POSE;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.startPose;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.SystemConstants.autoOnBlue;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.SystemConstants.telemetryAddLoopTime;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandBase.Constants.Enums;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Hardware;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.SystemData;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.ScoringSystem;
import org.firstinspires.ftc.teamcode.CommandBase.Util.TriggerManager;
import org.firstinspires.ftc.teamcode.OpModes.ExoMode;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

@Config
@TeleOp(name = "😈🔥", group = "main")
public class CrazyTeleOp extends ExoMode {
    private double startLoopTime;

    private Hardware hardware;
    private SwerveDrive swerve;
    private ScoringSystem system;
    private Lift lift;

    private GamepadEx g1, g2;
    private final SoloTeleOp.InputBus in = new SoloTeleOp.InputBus();
    private TriggerManager intakeTriggers, shooterTriggers;

    private Thread swerveThread, gamepadThread;

    public static double shooterP = Shooter.kP, shooterF = Shooter.kF;

    public static double ANGLE_ADJUST = 0;
    public static double angle = 0;
    public static double power = 0;

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
                .add(Enums.OpMode.TELE_OP)
                .getLoopTime(true);


        // create gamepad triggers
        intakeTriggers = new TriggerManager()
                .addTrigger(() -> in.evToggleIntake.getAndSet(false), () -> {
                    if (system.intake.on && !system.intake.reversed) {
                        system.intake.off();
                        system.isIntakeEnabled = false;
                        system.indexer.off();
                    } else {
                        system.intake.on();
                        system.isIntakeEnabled = true;
                        system.indexer.on();
                    }})   // intake
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
                    system.shootSorted = true;
                    system.indexer.indexPattern();
                })                       // sort
                .addTrigger(() -> in.evShootUnsorted.getAndSet(false) && in.spinupShooter, () -> {
                    system.shootSequence();
                    system.shootSorted = false;
                }) // shoot
                .addTrigger(() -> in.evHomeIndexer.getAndSet(false), () -> system.indexer.home());      // emergency homing


        // set the right start position
        try { Thread.sleep(150); } catch (InterruptedException e) {}
        hardware.localizer.setPositionEstimate(startPose);
        try { Thread.sleep(150); } catch (InterruptedException e) {}


        // initialize threads
        swerveThread = new Thread(() -> {
            while (opModeIsActive()) {
                swerve.update(new Pose(
                        in.ly,
                        -in.lx,
                        -in.rx)
                );

                swerve.lockHeadingToGoal(in.lockToGoal);
                if (in.evLockX.getAndSet(false)) swerve.setLockedX(true);
                if (in.evStartLift.getAndSet(false)) lift.on();
                if (in.evResetFieldCentric.getAndSet(false)) hardware.localizer.setPositionEstimate(new Pose(POSE.x, POSE.y, 0));

                swerve.write();
                lift.write();


                hardware.telemetry.addData("x", POSE.x);
                hardware.telemetry.addData("y", POSE.y);
                hardware.telemetry.addData("head", POSE.heading);

                hardware.telemetry.addData("art", system.indexer.elements.toString());
                hardware.telemetry.addData("shooter vel", system.shooter.wheelVelocity);
                hardware.telemetry.addData("shooter target", system.shooter.TARGET);
                updateTelemetry();

                Thread.yield();
            }
        }, "SwerveThread");
        gamepadThread = new Thread(() -> {
            while (opModeIsActive()) {
                hardware.bulk.clearCache(Enums.Hubs.ALL);
                hardware.read(system);
                swerve.read();
                lift.read();

                g1.readButtons();
                g2.readButtons();

                system.shooter.setPIDF(shooterP, 0, 0, shooterF);

                // continuous snapshot
                in.ly2 = g2.getLeftY();
                in.ly = g1.getLeftY();
                in.lx = g1.getLeftX();
                in.rx = g1.getRightX();
                in.lt = g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
                in.lockToGoal = in.lt > 0.1;


                in.spinupShooter = g2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1;

                // edges -> events (one-shot)
                if (g2.wasJustPressed(GamepadKeys.Button.B)) in.evToggleIntake.set(true);
                if (g2.wasJustPressed(GamepadKeys.Button.A)) in.evReverseIntake.set(true);
                if (g2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) in.evShootSorted.set(true);
                if (g2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) in.evShootUnsorted.set(true);
                if (g2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) in.evHomeIndexer.set(true);
                if (g1.isDown(GamepadKeys.Button.X) && g1.isDown(GamepadKeys.Button.DPAD_RIGHT)) in.evStartLift.set(true);
                if (g1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) in.evResetFieldCentric.set(true);

                if (g1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1) in.evLockX.set(true);

                system.indexer.manual(in.ly2, 0.2);
                system.shooter.update();
                system.write();

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

        try { Thread.sleep(3); } catch (InterruptedException e) {}
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
