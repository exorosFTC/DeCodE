package org.firstinspires.ftc.teamcode.OpModes.Main.Autonomus;

import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoAngularD;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoAngularP;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoAngularVelocityMultiplier;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoLinearDx;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoLinearPx;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoLinearPy;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.TeleOpVelocityMultiplier;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.goalPosition;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.goalPositionRed;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.startPoseRedClose;
import static org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.Subsystems.Indexer.elements;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandBase.Constants.SystemConstants;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Hardware;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.ScoringSystem;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.SystemData;
import org.firstinspires.ftc.teamcode.CommandBase.Util.SensorsEx.LimelightEx;
import org.firstinspires.ftc.teamcode.OpModes.ExoMode;
import org.firstinspires.ftc.teamcode.CustomPathing.AutoDrive;
import org.firstinspires.ftc.teamcode.CustomPathing.Math.Geometry.Pose;

@Config
@Autonomous(name = "RedClose", group = "main", preselectTeleOp = "😈🔥")
public class RedClose extends ExoMode {
    private Hardware hardware;
    private SwerveDrive swerve;
    private ScoringSystem system;
    private AutoDrive auto;

    public static double angularP = AutoAngularP, angularD = AutoAngularD;
    public static double linearPx = AutoLinearPx, linearDx = AutoLinearDx;
    public static double linearPy = AutoLinearPy, linearDy = AutoLinearDx;

    public static double linearThreshold = 0.97;
    public static double angularThreshold = 6;

    public static double angularMultiplier = AutoAngularVelocityMultiplier;

    @Override
    protected void Init() {
        new SystemData()
                .add(SystemConstants.OpMode.AUTONOMOUS)
                .setAutoOnBlue(false);

        hardware = Hardware.getInstance(this);
        swerve = new SwerveDrive(this);
        system = new ScoringSystem(this);

        auto = new AutoDrive(this, swerve, system, startPoseRedClose);

        system.indexer.preload();
        //hardware.limelight.start();
        //hardware.limelight.setPipeline(LimelightEx.Pipeline.RANDOMIZATION);

        while (opModeInInit()) {
            auto.linearCx.setPID(linearPx, 0, linearDx);
            auto.linearCy.setPID(linearPy, 0, linearDy);
            auto.angularC.setPID(angularP, 0, angularD);

            TeleOpVelocityMultiplier = angularMultiplier;

            auto.setBusyThresholdLinear(linearThreshold);
            auto.setBusyThresholdAngular(Math.toRadians(angularThreshold));
        }
    }



    @Override
    protected void WhenStarted() {
        preload();
        firstLine();
        secondLine();
        thirdLine();
    }

    private void preload() {
        auto.driveTo(new Pose(70, -40, Math.toRadians(-44.5)), 0.6)
                .waitDrive(0.3)
                .moveSystem(() -> {system.shooter.on(); system.indexer.microAdjust(false); })
                .waitDrive(0.98, true)
                .waitMs(50)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.shooter.on);
    }

    private void firstLine() {
        auto.driveTo(new Pose(40, -39, Math.toRadians(270)), 0.5)
                .waitDrive(0.4)
                .moveSystem(() -> system.intake.on())
                .driveTo(new Pose(40, -128, Math.toRadians(270)), 0.5)
                .waitDrive(0.965)
                .waitMs(300)
                .driveTo(new Pose(16, -118, Math.toRadians(0)), 0.8)
                .waitDrive(0.7)
                .driveTo(new Pose(15, -135, Math.toRadians(0)), 0.8, 1500)
                .waitDrive(0.965, true)
                .waitMs(300)
                .driveTo(new Pose(70, -60, Math.toRadians(-44)), 0.75)
                .moveSystem(() -> {system.shooter.on(); system.indexer.microAdjust(false); })
                .waitDrive(0.95, true)
                .moveSystem(() -> system.intake.off())
                .waitMs(100)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.shooter.on);
    }

    private void secondLine() {
        auto.driveTo(new Pose(-23, -50, Math.toRadians(270)), 0.6)
                .waitDrive(0.8)
                .moveSystem(() -> system.intake.on())
                .driveTo(new Pose(-23, -156, Math.toRadians(270)), 0.5)
                .waitDrive(0.88)
                .waitMs(600)
                .driveTo(new Pose(-23, -60, Math.toRadians(270)), 0.8)
                .moveSystem(() -> {
                    system.intake.reverse();
                    try{ Thread.sleep(400); } catch (InterruptedException e) {}
                    system.intake.off();
                })
                .waitDrive(0.4)
                .driveTo(new Pose(70, -60, Math.toRadians(-43.5)), 0.8)
                .waitDrive(0.85)
                .moveSystem(() -> {system.shooter.on(); system.indexer.microAdjust(false); })
                .waitDrive(0.95, true)
                .waitMs(100)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.shooter.on);
    }

    private void thirdLine() {
        auto.driveTo(new Pose(-73, -50, Math.toRadians(270)), 0.8)
                .waitDrive(0.85)
                .moveSystem(() -> system.intake.on())
                .driveTo(new Pose(-83, -156, Math.toRadians(270)), 0.5)
                .waitDrive(0.88)
                .waitMs(600)
                .driveTo(new Pose(-83, -60, Math.toRadians(270)), 0.8)
                .moveSystem(() -> {
                    system.intake.reverse();
                    try{ Thread.sleep(400); } catch (InterruptedException e) {}
                    system.intake.off();
                })
                .waitDrive(0.4)
                .driveTo(new Pose(115, -40, Math.toRadians(-60)), 0.8)
                .waitDrive(0.9)
                .moveSystem(() -> {system.shooter.on(); system.indexer.microAdjust(false); })
                .waitDrive(0.95, true)
                .waitMs(50)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.shooter.on);
    }

    private void leave() {
        auto.driveTo(new Pose(135, -60, Math.toRadians(-90)), 0.8)
                .waitDrive(0.9);
    }



    @Override
    protected void Loop() {}

}