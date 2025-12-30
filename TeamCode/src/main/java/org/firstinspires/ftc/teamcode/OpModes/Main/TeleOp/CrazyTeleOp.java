package org.firstinspires.ftc.teamcode.OpModes.Main.TeleOp;


import static androidx.core.math.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AngularD;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AngularP;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.K_STATIC;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.POSE;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.goalPosition;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.goalPositionBlue;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.goalPositionRed;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.startPose;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.startPoseBlueClose;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.startPoseBlueFar;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.startPoseRedClose;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.startPoseRedFar;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.SystemConstants.telemetryAddLoopTime;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.CommandBase.Constants.Enums;
import org.firstinspires.ftc.teamcode.CommandBase.Constants.SystemConstants;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Hardware;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.SystemData;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.ScoringSystem;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.Subsystems.Shooter;
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

    public static double ANGLE_ADJUST = Shooter.ANGLE_ADJUST;
    public static double angle = 0;
    public static double power = 0;

    public static double kStatic = K_STATIC;


    public static double moduleP = DriveConstants.swerveP, moduleD = DriveConstants.swerveD;
    public static double swerveP = AngularP, swerveD = AngularD;
    public static double shooterP = Shooter.kP, shooterD = Shooter.kD, shooterI = Shooter.kI, shooterF = Shooter.kF;

    public static String poseCase = "DEFAULT";

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
                .getLoopTime(true);

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
                .addTrigger(() -> g2.wasJustPressed(GamepadKeys.Button.DPAD_UP), () -> {
                    if (system.shooter.on) {
                        system.shooter.off();
                        g2.gamepad.rumble(0, 200, 400);
                    } else {
                        system.shooter.on();
                        g2.gamepad.rumble(200, 0, 100);
                    }
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
                //.addTrigger(() -> g2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER),   // add these in both cases
                //        () -> system.indexer.index(1))
                .addTrigger(() -> g2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER),
                        () -> system.indexer.home())
                .addTrigger(() -> g2.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON),
                        () -> system.indexer.sideswipe(3, true));

        swerveTriggers = new TriggerManager()
                .addTrigger(() -> g1.wasJustPressed(GamepadKeys.Button.DPAD_UP),
                        () -> swerve.setLockedX(true));


        try { Thread.sleep(150); } catch (InterruptedException e) {}
        hardware.localizer.setPositionEstimate(startPose);
        try { Thread.sleep(150); } catch (InterruptedException e) {}

        hardware.telemetry.addLine("INIT READY 😈😈😈");
        hardware.telemetry.update();
    }

    @Override
    protected void WaitForStart() {
        while (opModeInInit()) {
            g2.readButtons();

            if (g2.wasJustPressed(GamepadKeys.Button.A)) {
                try { Thread.sleep(150); } catch (InterruptedException e) {}
                POSE = startPoseRedClose;
                swerve.targetHeading = startPoseRedClose.heading;
                goalPosition = goalPositionRed;
                hardware.localizer.setPositionEstimate(startPoseRedClose);
                SystemConstants.autoOnBlue = false;
                poseCase = "RED CLOSE";
                try { Thread.sleep(150); } catch (InterruptedException e) {}

            } else if (g2.wasJustPressed(GamepadKeys.Button.B)) {
                try { Thread.sleep(150); } catch (InterruptedException e) {}
                POSE = startPoseRedFar;
                swerve.targetHeading = startPoseRedFar.heading;
                goalPosition = goalPositionRed;
                hardware.localizer.setPositionEstimate(startPoseRedFar);
                SystemConstants.autoOnBlue = false;
                poseCase = "RED FAR";
                try { Thread.sleep(150); } catch (InterruptedException e) {}

            } else if (g2.wasJustPressed(GamepadKeys.Button.X)) {
                try { Thread.sleep(150); } catch (InterruptedException e) {}
                POSE = startPoseBlueClose;
                swerve.targetHeading = startPoseBlueClose.heading;
                goalPosition = goalPositionBlue;
                hardware.localizer.setPositionEstimate(startPoseBlueClose);
                SystemConstants.autoOnBlue = true;
                poseCase = "BLUE CLOSE";
                try { Thread.sleep(150); } catch (InterruptedException e) {}

            } else if (g2.wasJustPressed(GamepadKeys.Button.Y)) {
                try { Thread.sleep(150); } catch (InterruptedException e) {}
                POSE = startPoseBlueFar;
                swerve.targetHeading = startPoseBlueFar.heading;
                goalPosition = goalPositionBlue;
                hardware.localizer.setPositionEstimate(startPoseBlueFar);
                SystemConstants.autoOnBlue = true;
                poseCase = "BLUE FAR";
                try { Thread.sleep(150); } catch (InterruptedException e) {}
            }

            hardware.telemetry.addLine("Case: " + poseCase);
            hardware.telemetry.update();
        }
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
                        swerve.headLim.calculate(-g1.getRightX() * 0.1))
                );

                swerveTriggers.check();

                //system.shooter.targetAngle = clamp(angle - (system.shooter.TARGET - system.shooter.wheelVelocity) * ANGLE_ADJUST, 0, angle);
                system.shooter.targetPower = power;

                K_STATIC = kStatic;
                swerve.setHeadingPID(swerveP, 0, swerveD);
                swerve.setModulePID(moduleP, 0, moduleD);
                swerve.lockHeadingToGoal(g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1);

                hardware.telemetry.addData("x", POSE.x);
                hardware.telemetry.addData("y", POSE.y);
                hardware.telemetry.addData("head", Math.toDegrees(POSE.heading));
                hardware.telemetry.addData("velocity", system.shooter.wheelVelocity);
                hardware.telemetry.addData("velocity target", system.shooter.TARGET);
                hardware.telemetry.addData("power", system.shooter.POWER);
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
        system.shooter.setPIDF(shooterP, shooterI, shooterD, shooterF);

        try { Thread.sleep(5); } catch (InterruptedException e) {}
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
