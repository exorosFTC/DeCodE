package org.firstinspires.ftc.teamcode.OpModes.Main.Autonomus;

import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.startPoseBlueClose;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandBase.Constants.SystemConstants;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Hardware;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.ScoringSystem;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.SystemData;
import org.firstinspires.ftc.teamcode.CommandBase.Util.SensorsEx.LimelightEx;
import org.firstinspires.ftc.teamcode.OpModes.ExoMode;
import org.firstinspires.ftc.teamcode.CustomPathing.AutoDrive;
import org.firstinspires.ftc.teamcode.CustomPathing.Math.Geometry.Pose;

@Config
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

        hardware.limelight.start();
        hardware.limelight.setPipeline(LimelightEx.Pipeline.RANDOMIZATION);

        system.indexer.preload();
    }



    @Override
    protected void WhenStarted() {
        preload();
        firstLine();
        //secondLine();
        leave();
        //thirdLine();

        auto.end();
    }



    private void preload() {
        auto.driveTo(new Pose(75, 60, Math.toRadians(-340)), 30)
                .moveSystem(() -> system.indexer.home())
                .waitDrive(() -> hardware.limelight.getRandomization(), 0.7, true)
                .moveSystem(() -> {
                    system.shooter.on();
                })
                .driveTo(new Pose(75, 60, Math.toRadians(-340)), 30)
                .moveSystem(() -> swerve.lockHeadingToGoal(true))
                .waitMs(900)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.indexer.isBusy())
                .moveSystem(() -> swerve.lockHeadingToGoal(false));
    }

    private void firstLine() {
        auto.driveTo(new Pose(35, 60, Math.toRadians(-270)), 30)
                .moveSystem(() -> system.indexer.home())
                .waitDrive(0.85)
                .driveTo(new Pose(35, 129, Math.toRadians(-270)), 20)
                .moveSystem(() -> system.intake.on())
                .waitDrive(0.88)
                .driveTo(new Pose(84, 60, Math.toRadians(-340)), 30, 2000)
                .moveSystem(() -> {
                    system.intake.reverse();
                    try{ Thread.sleep(400); } catch (InterruptedException e) {}
                    system.intake.off();
                })
                .waitDrive(0.82)
                .moveSystem(() -> {
                    //system.indexer.indexPattern();
                    system.shooter.on();
                })
                .moveSystem(() -> swerve.lockHeadingToGoal(true))
                .waitMs(900)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.indexer.isBusy())
                .moveSystem(() -> swerve.lockHeadingToGoal(false));
    }

    private void secondLine() {
        auto.driveTo(new Pose(-15, 75, Math.toRadians(-270)), 30)
                //.moveSystem(() -> system.indexer.home())
                .waitDrive(0.9)
                .moveSystem(() -> system.intake.on())
                .driveTo(new Pose(-15, 156, Math.toRadians(-270)), 30)
                .waitDrive(0.87)
                .driveTo(new Pose(-15, 60, Math.toRadians(-270)), 30, 2000)
                .moveSystem(() -> {
                    system.intake.reverse();
                    try{ Thread.sleep(400); } catch (InterruptedException e) {}
                    system.intake.off();
                })
                .waitDrive(0.7)
                .driveTo(new Pose(85, 60, Math.toRadians(-330)), 30)
                .waitDrive(0.7)
                .moveSystem(() -> {
                    //system.indexer.indexPattern();
                    system.shooter.on();
                })
                .waitDrive(0.9)
                .moveSystem(() -> swerve.lockHeadingToGoal(true))
                .waitMs(900)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.indexer.isBusy())
                .moveSystem(() -> swerve.lockHeadingToGoal(false));
    }

    private void thirdLine() {
        auto.driveTo(new Pose(-73, 60, Math.toRadians(-270)), 30)
                .waitDrive(0.95)
                .moveSystem(() -> system.intake.on())
                .driveTo(new Pose(-83, 156, Math.toRadians(-270)), 30)
                .waitDrive(0.87)
                .driveTo(new Pose(-83, 60, Math.toRadians(-270)), 30, 2000)
                .moveSystem(() -> {
                    system.intake.reverse();
                    try{ Thread.sleep(400); } catch (InterruptedException e) {}
                    system.intake.off();
                })
                .waitDrive(0.7)
                .driveTo(new Pose(135, 60, Math.toRadians(-315)), 30)
                .waitDrive(0.8)
                .moveSystem(() -> {
                    //system.indexer.indexPattern();
                    system.shooter.on();
                })
                .waitDrive(0.9)
                .moveSystem(() -> swerve.lockHeadingToGoal(true))
                .waitMs(900)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.indexer.isBusy())
                .moveSystem(() -> swerve.lockHeadingToGoal(false));
    }

    private void leave() {
        auto.driveTo(new Pose(135, 60, Math.toRadians(75)), 30)
                .waitDrive(0.9);
    }


    @Override
    protected void Loop() {}
}
