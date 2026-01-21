package org.firstinspires.ftc.teamcode.OpModes.Main.TeleOp;

import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.POSE;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.TeleOpAngularD;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.TeleOpAngularP;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.TeleOpLimelightD;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.TeleOpLimelightP;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.TeleOpVelocityMultiplier;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.goalPosition;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.goalPositionBlue;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.startPose;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.CommandBase.Constants.SystemConstants;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.Subsystems.Lift;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Hardware;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.SystemData;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.ScoringSystem;
import org.firstinspires.ftc.teamcode.CommandBase.Util.InputBus;
import org.firstinspires.ftc.teamcode.CommandBase.Util.SensorsEx.HubBulkRead;
import org.firstinspires.ftc.teamcode.CommandBase.Util.TriggerManager;
import org.firstinspires.ftc.teamcode.OpModes.ExoMode;
import org.firstinspires.ftc.teamcode.CustomPathing.Math.Geometry.Pose;

@Config
@TeleOp(name = "🍪", group = "main")
public class SoloTeleOp extends ExoMode {
    private Hardware hardware;
    private SwerveDrive swerve;
    private ScoringSystem system;
    private Lift lift;

    private GamepadEx g1;
    private final InputBus in = new InputBus();
    private TriggerManager intakeTriggers, shooterTriggers;

    private Thread swerveThread, gamepadThread;

    public static double velocityMultiplier = TeleOpVelocityMultiplier;
    public static double swerveP = TeleOpAngularP, swerveD = TeleOpAngularD, swervePmultiplier = TeleOpVelocityMultiplier;
    public static double moduleP = DriveConstants.swerveModuleP, moduleD = DriveConstants.swerveModuleD;

    public static double limelightP = TeleOpLimelightP, limelightD = TeleOpLimelightD;
    public static double angleAdjust = 0;
    public static double angle = 0.95;
    public static double power = 0;

    public static double shooterP = Shooter.kP, shooterF = Shooter.kF;


    @Override
    protected void Init() {
        // init hardware
        hardware = Hardware.getInstance(this);
        g1 = new GamepadEx(gamepad1);

        swerve = new SwerveDrive(this);
        system = new ScoringSystem(this);
        lift = new Lift(this);

        new SystemData()
                .add(SystemConstants.OpMode.TELE_OP)
                .setAutoOnBlue(false);


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
                .addTrigger(() -> in.evSort.getAndSet(false), () -> system.indexer.indexPattern())                       // sort
                .addTrigger(() -> in.evShoot.getAndSet(false) && in.spinupShooter, () -> system.shootSequence()) // shoot
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
                        -in.rx * 0.85)
                );

                swerve.setLimelightPID(limelightP, 0, limelightD);
                swerve.setHeadingPID(swerveP, 0, swerveD);
                swerve.setModulePID(moduleP, 0, moduleD);

                TeleOpVelocityMultiplier = velocityMultiplier;
                TeleOpAngularP = swerveP;
                swerve.setHeadingPID(swerveP, 0, swerveD);

                swerve.lockHeadingToGoal(in.lockToGoal);
                if (in.evLockX.getAndSet(false)) swerve.setLockedX(true);
                if (in.evResetHeading.getAndSet(false)) hardware.localizer.setPositionEstimate(new Pose(POSE.x, POSE.y, 0));
                if (in.evStartLift.getAndSet(false)) {
                    swerve.disable();
                    system.indexer.off();
                    system.intake.off();
                    system.shooter.off();

                    lift.on();
                }

                swerve.write();
                lift.write();


                //hardware.telemetry.addData("x", POSE.x);
                //hardware.telemetry.addData("y", POSE.y);
                //hardware.telemetry.addData("head", Math.toDegrees(POSE.heading));
                //hardware.telemetry.addData("distance", system.shooter.distance);
                //hardware.telemetry.addData("indexer pos", system.indexer.indexerPosition);
                //hardware.telemetry.addData("indexer target", system.indexer.target);
                //hardware.telemetry.addData("swerve rotation pow", swerve.leftBackModule.servoPower);
                //hardware.telemetry.addData("current state", swerve.leftBackModule.getModuleRotation());
                //hardware.telemetry.addData("target state", swerve.leftBackModule.getTargetRotation());

                hardware.telemetry.addData("vel", in.ly);
                //hardware.telemetry.addData("LF current", hardware.LeftFront.getCurrent(CurrentUnit.AMPS));
                //hardware.telemetry.addData("LB current", hardware.LeftBack.getCurrent(CurrentUnit.AMPS));
                //hardware.telemetry.addData("RF current", hardware.RightFront.getCurrent(CurrentUnit.AMPS));
                //hardware.telemetry.addData("RB current", hardware.RightBack.getCurrent(CurrentUnit.AMPS));

                //hardware.telemetry.addData("art", system.indexer.elements.toString());
                hardware.telemetry.addData("shooter velocity", system.shooter.correctedVelocity);
                hardware.telemetry.addData("shooter target", system.shooter.targetVelocity);
                hardware.updateTelemetry();

                Thread.yield();

            }
        }, "SwerveThread");
        gamepadThread = new Thread(() -> {
            while (opModeIsActive()) {
                hardware.bulk.clearCache(HubBulkRead.Hubs.ALL);
                hardware.read(system);
                swerve.read();
                lift.read();

                g1.readButtons();

                // continuous snapshot
                in.ly = g1.getLeftY();
                in.lx = g1.getLeftX();
                in.rx = g1.getRightX();
                in.lt = g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
                in.rt = g1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
                in.lockToGoal = in.lt > 0.1;
                in.spinupShooter = in.rt > 0.1;

                // edges -> events (one-shot)
                if (g1.wasJustPressed(GamepadKeys.Button.B)) in.evToggleIntake.set(true);
                if (g1.wasJustPressed(GamepadKeys.Button.A)) in.evReverseIntake.set(true);
                if (g1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) in.evSort.set(true);
                if (g1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) in.evShoot.set(true);
                if (g1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) in.evHomeIndexer.set(true);
                if (g1.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) in.evLockX.set(true);
                if (g1.isDown(GamepadKeys.Button.X) && g1.isDown(GamepadKeys.Button.DPAD_RIGHT)) in.evStartLift.set(true);
                if (g1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) in.evResetHeading.set(true);

                Shooter.ANGLE_ADJUST = angleAdjust;
                system.shooter.setPIDF(shooterP, 0, 0, shooterF);
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

        try { Thread.sleep(3); } catch (InterruptedException e) {}
    }
}
