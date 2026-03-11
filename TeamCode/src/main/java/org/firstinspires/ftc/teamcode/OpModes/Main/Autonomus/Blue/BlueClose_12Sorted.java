package org.firstinspires.ftc.teamcode.OpModes.Main.Autonomus.Blue;

import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.startPoseBlueClose;
import static org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.Subsystems.Indexer.elements;

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

@Autonomous(group = "blue", preselectTeleOp = "😈🔥")
public class BlueClose_12Sorted extends ExoMode {
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
        system.setTransferArm(false);
    }



    @Override
    protected void WhenStarted() {
        preload();
        readRandomization();
        firstLine();
        secondLine();
        thirdLine();

        system.shooter.off();
    }

    private void preload() {
        auto.driveTo(new Pose(124, 40, Math.toRadians(65)), 0.6)
                .moveSystem(() -> system.shooter.on())
                .lockHeadingToGoal(true)
                .waitDrive(0.8)
                .waitMs(450)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.isShooting)
                .moveSystem(() -> system.setTransferArm(true))
                .lockHeadingToGoal(false);
    }

    private void readRandomization() {
        hardware.limelight.start();
        hardware.limelight.setPipeline(LimelightEx.Pipeline.RANDOMIZATION);

        auto.driveTo(new Pose(144, 35, Math.toRadians(-50)), 0.8)
                .waitDrive(() -> { hardware.limelight.read(); hardware.limelight.getRandomization(); }, 0.9, false)
                .waitMs(() -> { hardware.limelight.read(); hardware.limelight.getRandomization(); }, 50)
                .moveSystem(() -> hardware.limelight.stop());
    }

    private void firstLine() {
        auto.moveSystem(() -> system.intake.on())
                .driveTo(new Pose(40, 50, Math.toRadians(90)), 0.6)
                .waitDrive(0.7)
                .driveTo(new Pose(40, 128, Math.toRadians(90)), 0.4)
                .waitDriveActionFailSafe(0.965, false,  () -> system.isIndexerFull)
                .driveTo(new Pose(28, 120, Math.toRadians(-10)), 0.8, 1000)
                .moveSystem(() ->
                        new Thread(() -> {
                            elements.set(0, Indexer.Artifact.GREEN);
                            elements.set(1, Indexer.Artifact.PURPLE);
                            elements.set(2, Indexer.Artifact.PURPLE);

                            system.intake.reverse();
                            try{ Thread.sleep(200); } catch (InterruptedException e) {}
                            system.indexer.indexPattern();
                            system.intake.off();

                            new Thread(() -> system.load()).start();
                        }).start())

                .waitDrive(0.7)
                .driveTo(new Pose(12, 140, Math.toRadians(-10)), 0.8, 1000)
                .waitDrive(0.9)


                .driveTo(new Pose(60, 60, Math.toRadians(38.5)), 0.6)
                .waitDrive(0.2)
                .lockHeadingToGoal(true)
                .waitDrive(0.97, false)
                .waitMs(450)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.isShooting)
                .moveSystem(() -> system.setTransferArm(true))
                .lockHeadingToGoal(false);
    }

    private void secondLine() {
        auto.moveSystem(() -> system.intake.on())
                .driveTo(new Pose(-20, 42, Math.toRadians(90)), 0.7)
                .waitDrive(0.6)
                .driveTo(new Pose(-20, 156, Math.toRadians(90)), 0.4, 2500)
                .waitDriveActionFailSafe(0.88, false, () -> system.isIndexerFull)
                .driveTo(new Pose(-20, 60, Math.toRadians(90)), 0.7)
                .moveSystem(() -> system.readSensors())
                .waitDrive(0.4)
                .moveSystem(() ->
                        new Thread(() -> {
                            try{ Thread.sleep(200); } catch (InterruptedException e) {}
                            system.intake.reverse();
                            try{ Thread.sleep(200); } catch (InterruptedException e) {}
                            system.indexer.indexPattern();
                            system.intake.off();

                            new Thread(() -> system.load()).start();
                        }).start())
                .driveTo(new Pose(60, 60, Math.toRadians(38.5)), 0.6)
                .waitDrive(0.3)
                .lockHeadingToGoal(true)
                .waitDrive(0.97, false)
                .waitMs(450)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.isShooting)
                .moveSystem(() -> system.setTransferArm(true))
                .lockHeadingToGoal(false);
    }

    private void thirdLine() {
        auto.moveSystem(() -> system.intake.on())
                .driveTo(new Pose(-75, 42, Math.toRadians(90)), 0.7)
                .waitDrive(0.6)
                .driveTo(new Pose(-75, 156, Math.toRadians(90)), 0.4, 2500)
                .waitDriveActionFailSafe(0.88, false, () -> system.isIndexerFull)
                .driveTo(new Pose(-75, 60, Math.toRadians(90)), 0.7)
                .moveSystem(() -> system.readSensors())
                .moveSystem(() -> system.readSensors())
                .waitDrive(0.4)
                .moveSystem(() ->
                        new Thread(() -> {
                            try{ Thread.sleep(200); } catch (InterruptedException e) {}
                            system.intake.reverse();
                            try{ Thread.sleep(200); } catch (InterruptedException e) {}
                            system.indexer.indexPattern();
                            system.intake.off();

                            new Thread(() -> system.load()).start();
                        }).start())
                .driveTo(new Pose(90, 40, Math.toRadians(54)), 0.6)                .waitDrive(0.3)
                .lockHeadingToGoal(true)
                .waitDrive(0.97, false)
                .waitMs(450)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.isShooting)
                .moveSystem(() -> system.setTransferArm(true))
                .lockHeadingToGoal(false);
    }



    @Override
    protected void Loop() {}
}