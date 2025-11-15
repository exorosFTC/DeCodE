package org.firstinspires.ftc.teamcode.OpModes.Main.TeleOp;


import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AngularD;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AngularP;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.startPose;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.SystemConstants.telemetryAddLoopTime;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.CommandBase.Constants.Enums;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Hardware;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.SystemData;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.ScoringSystem;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Swerve.SwerveKinematics;
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

    public static double moduleP = DriveConstants.swerveP, moduleD = DriveConstants.swerveD;
    public static double swerveP = AngularP, swerveD = AngularD;
    public static double shooterP = Shooter.kP, shooterD = Shooter.kD, shooterI = Shooter.kI;

    @Override
    protected void Init() {
        hardware = Hardware.getInstance(this);
        g1 = new GamepadEx(gamepad1);
        g2 = new GamepadEx(gamepad2);

        swerve = new SwerveDrive(this);
        system = new ScoringSystem(this);

        new SystemData()
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
                            () -> {
                                system.intake.reverse();
                                system.indexer.on();
                            });

        shooterTriggers = new TriggerManager()
                .addTrigger(() -> g2.wasJustPressed(GamepadKeys.Button.DPAD_UP),
                            () -> {
                            if (system.shooter.on)
                                system.shooter.off();
                            else system.shooter.on();
                            })
                .addTrigger(() -> g2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT),
                        () -> {
                            system.indexer.setRapidFire(false);
                            system.shootSequence();
                        })
                .addTrigger(() -> g2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT),
                            () -> {
                            system.indexer.setRapidFire(true);
                            system.shootSequence();
                            })
                .addTrigger(() -> g2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER),   // add these in both cases
                        () -> system.indexer.index(1))
                .addTrigger(() -> g2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER),
                        () -> system.indexer.home())
                .addTrigger(() -> g2.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON),
                        () -> system.indexer.sideswipe(3, true));

        swerveTriggers = new TriggerManager()
                .addTrigger(() -> g1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER),
                        () -> swerve.setMode(Enums.SwerveMode.SPORT))
                .addTrigger(() -> g1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER),
                        () -> swerve.setMode(Enums.SwerveMode.ECHO))
                .addTrigger(() -> g1.wasJustPressed(GamepadKeys.Button.DPAD_UP),
                        () -> swerve.setLockedX(!SwerveKinematics.isLockedX()))
                .addTrigger(() -> g1.wasJustPressed(GamepadKeys.Button.X),
                        () -> {
                            system.tilt.on();
                            try { Thread.sleep(800); } catch (InterruptedException e) {}
                            system.tilt.off();
                            hardware.led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                        });


        try { Thread.sleep(150); } catch (InterruptedException e) {}
        hardware.localizer.setPositionEstimate(startPose);
        try { Thread.sleep(150); } catch (InterruptedException e) {}

        hardware.led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
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
                        -g1.getLeftY(),
                        g1.getLeftX(),
                        g1.getRightX() * 0.2)
                );

                swerveTriggers.check();

                //system.shooter.targetAngle = angle - (system.shooter.TARGET - system.shooter.wheelVelocity - 5) * ANGLE_ADJUST;
                //system.shooter.targetPower = power;

                //swerve.setHeadingPID(swerveP, 0, swerveD);
                //swerve.setModulePID(moduleP, 0, moduleD);
                swerve.lockHeadingToGoal(g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1);

                hardware.telemetry.addData("velocity", system.shooter.wheelVelocity);
                hardware.telemetry.addData("velocity target", system.shooter.TARGET);
                hardware.telemetry.addData("angle", system.shooter.targetAngle);
                hardware.telemetry.addData("distance", system.shooter.distance);
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
        system.shooter.setPID(shooterP, shooterI, shooterD);

        try { Thread.sleep(15); } catch (InterruptedException e) {}
        //updateTelemetry();
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
