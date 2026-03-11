package org.firstinspires.ftc.teamcode.OpModes.Main.Autonomus.Blue;

import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.startPoseBlueClose;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandBase.Constants.SystemConstants;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Hardware;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.ScoringSystem;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.SystemData;
import org.firstinspires.ftc.teamcode.OpModes.ExoMode;
import org.firstinspires.ftc.teamcode.CustomPathing.AutoDrive;
import org.firstinspires.ftc.teamcode.CustomPathing.Math.Geometry.Pose;

@Autonomous(group = "blue", preselectTeleOp = "😈🔥")
public class BlueClose_15 extends ExoMode {
    private Hardware hardware;
    private SwerveDrive swerve;
    private ScoringSystem system;
    private AutoDrive auto;

    @Override
    protected void Init() {
        new SystemData()
                .add(SystemConstants.OpMode.AUTONOMOUS)
                .setVelocityTimeout(true)
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
        secondLine();
        gateCycle1();
        gateCycle2();
        firstLine();

        auto.lockHeadingToGoal(true);
        system.shooter.off();
    }

    private void preload() {
        auto.driveTo(new Pose(80, 60, Math.toRadians(45)), 0.45)
                .moveSystem(() -> system.shooter.on())
                .waitDrive(0.96)
                .lockHeadingToGoal(true)
                .waitMs(200)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.isShooting)
                .moveSystem(() -> system.setTransferArm(true))
                .lockHeadingToGoal(false);
    }

    private void firstLine() {
        auto.moveSystem(() -> system.intake.on())
                .driveTo(new Pose(40, 50, Math.toRadians(-270)), 0.8)
                .waitDrive(0.4)
                .driveTo(new Pose(40, 132, Math.toRadians(-270)), 0.6)
                .waitDriveActionFailSafe(0.965, false,  () -> system.isIndexerFull)
                .driveTo(new Pose(90, 40, Math.toRadians(48)), 0.7)
                .waitMs(300)
                .moveSystem(() -> {
                    new Thread(() -> system.load()).start();

                    try{ Thread.sleep(200); } catch (InterruptedException e) {}
                    system.intake.reverse();
                    try{ Thread.sleep(400); } catch (InterruptedException e) {}
                    system.intake.off();
                })
                .waitDrive(0.2)
                .lockHeadingToGoal(true)
                .waitDrive(0.96, false)
                .waitMs(300)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.isShooting)
                .moveSystem(() -> system.setTransferArm(true));
    }

    private void secondLine() {
        auto.driveTo(new Pose(-20, 42, Math.toRadians(-270)), 0.7, 1500)
                .waitMs(400)
                .moveSystem(() -> system.intake.on())
                .waitDrive(0.6, false)
                .driveTo(new Pose(-20, 156, Math.toRadians(-270)), 0.6, 2600)
                .waitDriveActionFailSafe(0.9, false, () -> system.isIndexerFull)
                .driveTo(new Pose(-20, 60, Math.toRadians(-270)), 0.8, 1500)
                .waitDrive(0.4, false)
                .moveSystem(() ->
                        new Thread(() -> {
                            new Thread(() -> system.load()).start();

                            try{ Thread.sleep(200); } catch (InterruptedException e) {}
                            system.intake.reverse();
                            try{ Thread.sleep(400); } catch (InterruptedException e) {}
                            system.intake.off();
                        }).start())
                .driveTo(new Pose(60, 60, Math.toRadians(44)), 0.8, 3000)
                .waitDrive(0.4, false)
                .lockHeadingToGoal(true)
                .waitDrive(0.96, false)
                .waitMs(300)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.isShooting)
                .moveSystem(() -> system.setTransferArm(true))
                .lockHeadingToGoal(false);
    }

    private void gateCycle1() {
        auto.driveTo(new Pose(-10, 50, Math.toRadians(90)), 0.8)
                .waitMs(400)
                .moveSystem(() -> system.intake.on())
                .waitDrive(0.4, false)
                .driveTo(new Pose(-10, 149, Math.toRadians(80)), 0.55, 800)
                .waitDrive(0.8, false)
                .driveTo(new Pose(-10, 149, Math.toRadians(80)), 0.4, 800)
                .waitDrive(0.73, false)
                .waitMs(100)

                .driveTo(new Pose(-40, 144, Math.toRadians(50)), 0.6, 1000)
                .waitDriveActionFailSafe(0.9, false, () -> system.isIndexerFull)
                .waitMsActionFailSafe(550, () -> system.isIndexerFull)
                .driveTo(new Pose(-25, 149, Math.toRadians(90)), 0.6, 500)
                .waitDriveActionFailSafe(0.9, false, () -> system.isIndexerFull)
                .driveTo(new Pose(-40, 149, Math.toRadians(90)), 0.6, 500)
                .waitDriveActionFailSafe(0.9, false, () -> system.isIndexerFull)

                .driveTo(new Pose(-23, 70, Math.toRadians(90)), 0.8)
                .moveSystem(() -> {
                    new Thread(() -> system.load()).start();

                    try{ Thread.sleep(200); } catch (InterruptedException e) {}
                    system.intake.reverse();
                    try{ Thread.sleep(400); } catch (InterruptedException e) {}
                    system.intake.off();
                })
                .waitDrive(0.4, false)
                .driveTo(new Pose(60, 60, Math.toRadians(42)), 0.7)
                .waitDrive(0.4, false)
                .lockHeadingToGoal(true)


                .waitDrive(0.96, false)
                .waitMs(400)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.isShooting)
                .lockHeadingToGoal(false);
    }

    private void gateCycle2() {
        auto.driveTo(new Pose(-10, 50, Math.toRadians(90)), 0.8)
                .waitMs(500)
                .moveSystem(() -> system.intake.on())
                .waitDrive(0.4, false)
                .driveTo(new Pose(-10, 149, Math.toRadians(80)), 0.55, 800)
                .waitDrive(0.8, false)
                .driveTo(new Pose(-10, 149, Math.toRadians(80)), 0.4, 800)
                .waitDrive(0.73, false)
                .waitMs(100)

                .driveTo(new Pose(-40, 144, Math.toRadians(50)), 0.6, 1000)
                .waitDriveActionFailSafe(0.9, false, () -> system.isIndexerFull)
                .waitMsActionFailSafe(550, () -> system.isIndexerFull)
                .driveTo(new Pose(-25, 149, Math.toRadians(90)), 0.6, 500)
                .waitDriveActionFailSafe(0.9, false, () -> system.isIndexerFull)
                .driveTo(new Pose(-40, 149, Math.toRadians(90)), 0.6, 500)
                .waitDriveActionFailSafe(0.9, false, () -> system.isIndexerFull)

                .driveTo(new Pose(-23, 70, Math.toRadians(90)), 0.8)
                .moveSystem(() -> {
                    new Thread(() -> system.load()).start();

                    try{ Thread.sleep(200); } catch (InterruptedException e) {}
                    system.intake.reverse();
                    try{ Thread.sleep(400); } catch (InterruptedException e) {}
                    system.intake.off();
                })
                .waitDrive(0.4, false)
                .driveTo(new Pose(60, 60, Math.toRadians(42)), 0.7)
                .waitDrive(0.4, false)
                .lockHeadingToGoal(true)

                .waitDrive(0.96, false)
                .waitMs(400)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.isShooting)
                .lockHeadingToGoal(false);
    }



    @Override
    protected void Loop() {}
}