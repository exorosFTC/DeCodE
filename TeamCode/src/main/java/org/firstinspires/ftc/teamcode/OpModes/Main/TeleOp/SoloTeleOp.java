package org.firstinspires.ftc.teamcode.OpModes.Main.TeleOp;


import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AngularD;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AngularP;
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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.CommandBase.Constants.Enums;
import org.firstinspires.ftc.teamcode.CommandBase.Constants.SystemConstants;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Hardware;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.SystemData;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.ScoringSystem;
import org.firstinspires.ftc.teamcode.CommandBase.Util.TriggerManager;
import org.firstinspires.ftc.teamcode.OpModes.ExoMode;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

@Config
@TeleOp(name = "🍪", group = "main")
public class SoloTeleOp extends ExoMode {
    private double startLoopTime;

    private Hardware hardware;
    private SwerveDrive swerve;
    private ScoringSystem system;

    private GamepadEx g1;
    public static double moduleP = DriveConstants.swerveP, moduleD = DriveConstants.swerveD;
    public static double swerveP = AngularP, swerveD = AngularD;
    private TriggerManager intakeTriggers, shooterTriggers;

    public static String poseCase = "DEFAULT";

    @Override
    protected void Init() {
        hardware = Hardware.getInstance(this);
        g1 = new GamepadEx(gamepad1);

        swerve = new SwerveDrive(this);
        system = new ScoringSystem(this);

        new SystemData()
                .add(Enums.OpMode.TELE_OP)
                .setAutoOnBlue(false)
                .getLoopTime(true);

        intakeTriggers = new TriggerManager()
                .addTrigger(() -> g1.wasJustPressed(GamepadKeys.Button.B),
                        () ->
                            new Thread(() -> {
                                if(system.intake.on) {
                                    system.intake.off();
                                    system.indexer.off();
                                }
                                else {
                                    system.intake.on();
                                    system.indexer.on();
                                }
                            }).start())
                .addTrigger(() -> g1.wasJustPressed(GamepadKeys.Button.A),
                        () ->
                            new Thread(() -> {
                                system.intake.reverse();
                                system.indexer.on();
                            }).start());

        shooterTriggers = new TriggerManager()
                .addTrigger(() -> g1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER), () -> new Thread(() -> system.shooter.on()).start() )
                .addTrigger(() -> g1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT),
                        () ->
                            new Thread(() -> {
                                system.shooter.on();
                                try { Thread.sleep(100); } catch (InterruptedException e) {};

                                system.indexer.setRapidFire(false);
                                system.shootSequence();
                            }).start())
                .addTrigger(() -> g1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER),
                        () ->
                            new Thread(() -> {
                                system.shooter.on();
                                try { Thread.sleep(100); } catch (InterruptedException e) {};

                                system.indexer.setRapidFire(true);
                                system.shootSequence();
                            }).start())
                .addTrigger(() -> g1.wasJustPressed(GamepadKeys.Button.DPAD_UP),
                        () -> new Thread(() -> system.indexer.home()).start());


        try { Thread.sleep(150); } catch (InterruptedException e) {}
        hardware.localizer.setPositionEstimate(startPose);
        try { Thread.sleep(150); } catch (InterruptedException e) {}

        hardware.telemetry.addLine("INIT READY 😈😈😈");
        hardware.telemetry.update();
    }

    @Override
    protected void WaitForStart() {
        while (opModeInInit()) {
            g1.readButtons();

            if (g1.wasJustPressed(GamepadKeys.Button.A)) {
                try { Thread.sleep(150); } catch (InterruptedException e) {}
                POSE = startPoseRedClose;
                swerve.targetHeading = startPoseRedClose.heading;
                goalPosition = goalPositionRed;
                hardware.localizer.setPositionEstimate(startPoseRedClose);
                SystemConstants.autoOnBlue = false;
                poseCase = "RED CLOSE";
                try { Thread.sleep(150); } catch (InterruptedException e) {}

            } else if (g1.wasJustPressed(GamepadKeys.Button.B)) {
                try { Thread.sleep(150); } catch (InterruptedException e) {}
                POSE = startPoseRedFar;
                swerve.targetHeading = startPoseRedFar.heading;
                goalPosition = goalPositionRed;
                hardware.localizer.setPositionEstimate(startPoseRedFar);
                SystemConstants.autoOnBlue = false;
                poseCase = "RED FAR";
                try { Thread.sleep(150); } catch (InterruptedException e) {}

            } else if (g1.wasJustPressed(GamepadKeys.Button.X)) {
                try { Thread.sleep(150); } catch (InterruptedException e) {}
                POSE = startPoseBlueClose;
                swerve.targetHeading = startPoseBlueClose.heading;
                goalPosition = goalPositionBlue;
                hardware.localizer.setPositionEstimate(startPoseBlueClose);
                SystemConstants.autoOnBlue = true;
                poseCase = "BLUE CLOSE";
                try { Thread.sleep(150); } catch (InterruptedException e) {}

            } else if (g1.wasJustPressed(GamepadKeys.Button.Y)) {
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

        new Thread(() -> system.indexer.home()).start();
    }

    @Override
    protected void Loop() {
        hardware.read(system, swerve);
        g1.readButtons();

        swerve.setModulePID(moduleP, 0, moduleD);
        swerve.setHeadingPID(swerveP, 0, swerveD);

        system.update();
        swerve.update(new Pose(
                swerve.xLim.calculate(g1.getLeftY()),
                swerve.yLim.calculate(-g1.getLeftX()),
                swerve.headLim.calculate(-g1.getRightX() * 0.15))
        );

        if (system.isIntakeEnabled)
            intakeTriggers.check();
        shooterTriggers.check();

        //swerve.lockHeadingToGoal(g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1);

        hardware.telemetry.addData("velocity", system.shooter.wheelVelocity);
        hardware.telemetry.addData("velocity target", system.shooter.TARGET);
        hardware.telemetry.addData("angle", system.shooter.targetAngle);
        hardware.telemetry.addData("distance", system.shooter.distance);
        hardware.telemetry.addData("x", POSE.x);
        hardware.telemetry.addData("y", POSE.y);
        hardware.telemetry.addData("heading", POSE.heading);
        updateTelemetry();

        hardware.write(system, swerve);
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
