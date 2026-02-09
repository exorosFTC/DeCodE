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
        secondLine();
        //thirdLine();
        leave();

        auto.end();
    }

    private void preload() {
        auto.driveTo(new Pose(70, 40, Math.toRadians(-10)), 0, 30)
                .waitDrive(() -> { hardware.limelight.read(); hardware.limelight.getRandomization(); }, 0.95, true)
                .moveSystem(() -> hardware.limelight.stop())
                .driveTo(new Pose(70, 40, Math.toRadians(45)), 0, 30)
                .moveSystem(() -> system.indexer.indexPattern())
                .waitDrive(0, true)
                .waitAction(() -> !system.indexer.isIndexing)
                .moveSystem(() -> {system.shooter.on(); system.indexer.microAdjust(false); })
                .waitMs(600)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.shooter.on);
    }

    private void firstLine() {
        auto.driveTo(new Pose(35, 50, Math.toRadians(-270)), 0, 30)
                .waitDrive(0.93, true)
                .moveSystem(() -> system.intake.on())
                .driveTo(new Pose(35, 119, Math.toRadians(-270)), 0, 10)
                .waitDrive(0.97)
                .waitMs(400)
                .moveSystem(() -> {
                    system.intake.reverse();
                    try{ Thread.sleep(400); } catch (InterruptedException e) {}
                    system.intake.off();

                    elements.set(0, Indexer.Artifact.GREEN);
                    elements.set(1, Indexer.Artifact.PURPLE);
                    elements.set(2, Indexer.Artifact.PURPLE);
                })
                .driveTo(new Pose(70, 60, Math.toRadians(35)), 0, 30)
                .moveSystem(() -> system.indexer.indexPattern())
                .waitAction(() -> !system.indexer.isIndexing)
                .waitDrive(0.97)
                .moveSystem(() -> {system.shooter.on(); system.indexer.microAdjust(false); })
                .waitMs(600)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.shooter.on);
    }

    private void secondLine() {
        auto.driveTo(new Pose(-25, 75, Math.toRadians(-270)), 0, 30)
                .moveSystem(() -> system.indexer.home())
                .waitDrive(0.95, true)
                .moveSystem(() -> system.intake.on())
                .driveTo(new Pose(-25, 156, Math.toRadians(-270)), 0, 10)
                .waitDrive(0.87)
                .driveTo(new Pose(-25, 60, Math.toRadians(-270)), 0, 30)
                .moveSystem(() -> {
                    system.intake.reverse();
                    try{ Thread.sleep(400); } catch (InterruptedException e) {}
                    system.intake.off();

                    elements.set(0, Indexer.Artifact.PURPLE);
                    elements.set(1, Indexer.Artifact.GREEN);
                    elements.set(2, Indexer.Artifact.PURPLE);
                })
                .waitDrive(0.7)
                .driveTo(new Pose(70, 60, Math.toRadians(45)), 0, 30, 4000)
                .moveSystem(() -> system.indexer.indexPattern())
                .waitAction(() -> !system.indexer.isIndexing)
                .waitDrive(0.97, true)
                .moveSystem(() -> {system.shooter.on(); system.indexer.microAdjust(false); })
                .waitMs(600)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.shooter.on);
    }

    private void thirdLine() {
        auto.driveTo(new Pose(-73, 60, Math.toRadians(-270)), 0, 30)
                .waitDrive(0.95)
                .moveSystem(() -> system.intake.on())
                .driveTo(new Pose(-83, 156, Math.toRadians(-270)), 0, 15)
                .waitDrive(0.87)
                .driveTo(new Pose(-83, 60, Math.toRadians(-270)), 0, 30, 2000)
                .moveSystem(() -> {
                    system.intake.reverse();
                    try{ Thread.sleep(400); } catch (InterruptedException e) {}
                    system.intake.off();
                })
                .waitDrive(0.7)
                .driveTo(new Pose(150, 60, Math.toRadians(90)), 30, 30)
                .waitDrive(0.7)
                .moveSystem(() -> {
                    //system.indexer.indexPattern();
                    system.shooter.on();
                })
                .waitDrive(0.98, true)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.indexer.isBusy());
    }

    private void leave() {
        auto.driveTo(new Pose(135, 60, Math.toRadians(90)), 30, 30)
                .waitDrive(0.9);
    }



    @Override
    protected void Loop() {}
}
