package org.firstinspires.ftc.teamcode.OpModes.Main.Autonomus;

import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.startPoseBlueClose;
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

@Autonomous(name = "BlueClose", group = "main", preselectTeleOp = "😈🔥")
public class BlueClose extends ExoMode {
    private Hardware hardware;
    private SwerveDrive swerve;
    private ScoringSystem system;
    private AutoDrive auto;

    @Override
    protected void Init() {
        new SystemData()
                .add(SystemConstants.OpMode.AUTONOMOUS)
                .setAutoOnBlue(true);

        hardware = Hardware.getInstance(this);
        swerve = new SwerveDrive(this);
        system = new ScoringSystem(this);

        auto = new AutoDrive(this, swerve, system, startPoseBlueClose);

        system.indexer.preload();
    }



    @Override
    protected void WhenStarted() {
        preload();
        firstLine();
        secondLine();
        thirdLine();
    }

    private void preload() {
        auto.driveTo(new Pose(70, 40, Math.toRadians(46)), 0.6)
                .waitDrive(0.3)
                .moveSystem(() -> {system.shooter.on(); system.indexer.microAdjust(false); })
                .waitDrive(0.9, true)
                .waitMs(50)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.shooter.on);
    }

    private void firstLine() {
        auto.driveTo(new Pose(40, 39, Math.toRadians(-270)), 0.5)
                .waitDrive(0.4)
                .moveSystem(() -> system.intake.on())
                .driveTo(new Pose(40, 128, Math.toRadians(-270)), 0.45)
                .waitDrive(0.965)
                .waitMs(300)
                .driveTo(new Pose(16, 118, Math.toRadians(0)), 0.8)
                .waitDrive(0.7)
                .driveTo(new Pose(15, 135, Math.toRadians(0)), 0.8, 1500)
                .waitDrive(0.965, true)
                .waitMs(300)
                .driveTo(new Pose(70, 60, Math.toRadians(46)), 0.75)
                .moveSystem(() -> system.intake.off())
                .moveSystem(() -> {system.shooter.on(); system.indexer.microAdjust(false); })
                .waitDrive(0.95, true)
                .waitMs(10)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.shooter.on);
    }

    private void secondLine() {
        auto.driveTo(new Pose(-23, 50, Math.toRadians(-270)), 0.6)
                .waitDrive(0.8)
                .moveSystem(() -> system.intake.on())
                .driveTo(new Pose(-23, 156, Math.toRadians(-270)), 0.45)
                .waitDrive(0.88)
                .waitMs(600)
                .driveTo(new Pose(-23, 60, Math.toRadians(-270)), 0.8)
                .moveSystem(() -> {
                    system.intake.reverse();
                    try{ Thread.sleep(400); } catch (InterruptedException e) {}
                    system.intake.off();
                })
                .waitDrive(0.4)
                .driveTo(new Pose(70, 60, Math.toRadians(40)), 0.8)
                .waitDrive(0.9)
                .moveSystem(() -> {system.shooter.on(); system.indexer.microAdjust(false); })
                .waitDrive(0.95, true)
                .waitMs(50)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.shooter.on);
    }

    private void thirdLine() {
        auto.driveTo(new Pose(-73, 50, Math.toRadians(-270)), 0.8)
                .waitDrive(0.85)
                .moveSystem(() -> system.intake.on())
                .driveTo(new Pose(-83, 156, Math.toRadians(-270)), 0.45)
                .waitDrive(0.88)
                .waitMs(600)
                .driveTo(new Pose(-83, 60, Math.toRadians(-270)), 0.8)
                .moveSystem(() -> {
                    system.intake.reverse();
                    try{ Thread.sleep(400); } catch (InterruptedException e) {}
                    system.intake.off();
                })                 .waitDrive(0.4)
                .driveTo(new Pose(115, 40, Math.toRadians(60)), 0.8)
                .waitDrive(0.9)
                .moveSystem(() -> {system.shooter.on(); system.indexer.microAdjust(false); })
                .waitDrive(0.95, true)
                .waitMs(50)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.shooter.on);
    }

    private void leave() {
        auto.driveTo(new Pose(135, 60, Math.toRadians(90)), 0.8)
                .waitDrive(0.9);
    }



    @Override
    protected void Loop() {}
}