package org.firstinspires.ftc.teamcode.OpModes.Main.TeleOp;


import static androidx.core.math.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AngularD;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AngularP;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.POSE;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.startPose;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.SystemConstants.telemetryAddLoopTime;
import static org.firstinspires.ftc.teamcode.Pathing.Math.ShootingZoneIntersection.isInShootingZone;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.CommandBase.Constants.Enums;
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

    private GamepadEx g1, g2;
    private TriggerManager intakeTriggers, shooterTriggers, swerveTriggers;

    private static final Gamepad.RumbleEffect SHOOTER_ON = new Gamepad.RumbleEffect.Builder()
            .addStep(1.0, 0.0, 120)
            .addStep(0.0, 0.0, 60)
            .addStep(1.0, 0.0, 120)   // left double-tap = ON
            .build();

    private static final Gamepad.RumbleEffect SHOOTER_OFF = new Gamepad.RumbleEffect.Builder()
            .addStep(0.0, 1.0, 250)   // long right buzz = OFF
            .build();

    public static double ANGLE_ADJUST = 0;
    public static double angle = 0;
    public static double power = 0;

    public static double swerveP = AngularP, swerveD = AngularD;

    @Override
    protected void Init() {
        // init hardware
        hardware = Hardware.getInstance(this);
        g1 = new GamepadEx(gamepad1);
        g2 = new GamepadEx(gamepad2);

        swerve = new SwerveDrive(this);
        system = new ScoringSystem(this);

        new SystemData()
                .add(Enums.OpMode.TELE_OP)
                .getLoopTime(true);


        // create gamepad triggers
        intakeTriggers = new TriggerManager()
                .addTrigger(() -> g2.wasJustPressed(GamepadKeys.Button.B),
                            () -> {
                            if (system.intake.on && !system.intake.reversed) {
                                system.intake.off();
                                system.indexer.off();
                            } else {
                                system.intake.on();
                                system.indexer.on();
                            }})
                .addTrigger(() -> g2.wasJustPressed(GamepadKeys.Button.A),
                            () -> {
                                if (system.intake.on && system.intake.reversed) {
                                    system.intake.off();
                                    system.indexer.off();
                                } else {
                                    system.intake.reverse();
                                    system.indexer.on();
                                }
                            });

        shooterTriggers = new TriggerManager()
                .addTrigger(() -> g2.wasJustPressed(GamepadKeys.Button.DPAD_UP), () -> {
                    if (!system.shooter.on) { system.shooter.on(); system.indexer.microAdjust(); }
                    else system.shooter.off();

                    g2.gamepad.runRumbleEffect(!system.shooter.on ? SHOOTER_ON : SHOOTER_OFF);
                })
                .addTrigger(() -> g2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT),
                        () -> {
                            while (!system.shooter.ready()) { system.update(); }

                            system.indexer.setRapidFire(false);
                            system.shootSequence();
                        })
                .addTrigger(() -> g2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT),
                            () -> {
                            while (!system.shooter.ready()) { system.update(); }

                            system.indexer.setRapidFire(true);
                            system.shootSequence();
                            })
                .addTrigger(() -> g2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER),
                        () -> system.indexer.index(1))
                .addTrigger(() -> g2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER),
                        () -> system.indexer.home())
                .addTrigger(() -> g2.wasJustPressed(GamepadKeys.Button.X),
                        () -> system.indexer.sideswipe(3, true));

        swerveTriggers = new TriggerManager();


        try { Thread.sleep(150); } catch (InterruptedException e) {}
        hardware.localizer.setPositionEstimate(startPose);
        try { Thread.sleep(150); } catch (InterruptedException e) {}

        hardware.telemetry.addLine("INIT READY 😈😈😈");
        hardware.telemetry.update();
    }

    @Override
    protected void WhenStarted() {
        hardware.telemetry.clearAll();

        new Thread(() -> {
            while (opModeIsActive()) {
                hardware.read(system, swerve);
                g1.readButtons();

                swerve.update(new Pose(
                        swerve.xLim.calculate(g1.getLeftY()),
                        swerve.yLim.calculate(-g1.getLeftX()),
                        swerve.headLim.calculate(-g1.getRightX() * 0.05))
                );

                swerveTriggers.check();

                //system.shooter.targetAngle = clamp(angle - (system.shooter.TARGET - system.shooter.wheelVelocity) * ANGLE_ADJUST, 0.27, 0.97);
                //system.shooter.targetPower = power;

                //swerve.setHeadingPID(swerveP, 0, swerveD);

                swerve.lockHeadingToGoal(g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1);
                swerve.setLockedX(g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1);

                // localization telemetry
                //hardware.telemetry.addData("x", POSE.x);
                //hardware.telemetry.addData("y", POSE.y);
                //hardware.telemetry.addData("head", Math.toDegrees(POSE.heading));
                //hardware.telemetry.addData("target head", Math.toDegrees(swerve.targetHeading));
                hardware.telemetry.addData("isInShootingZone", isInShootingZone());

                // shooter telemetry
                hardware.telemetry.addData("velocity", system.shooter.wheelVelocity);
                hardware.telemetry.addData("velocity target", system.shooter.TARGET);
                //hardware.telemetry.addData("distance", system.shooter.distance);

                // indexer telemetry
                //hardware.telemetry.addData("artifact list", system.indexer.elements.toString());

                updateTelemetry();
                hardware.write(system, swerve);
            }
        }).start();

        system.indexer.home();
    }

    @Override
    protected void Loop() {
        g2.readButtons();

        if (system.isIntakeEnabled)
            intakeTriggers.check();
        shooterTriggers.check();

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
}
