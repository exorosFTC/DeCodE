package org.firstinspires.ftc.teamcode.OpModes.Main.Autonomus.Red;

import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.startPoseRedFar;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandBase.Constants.SystemConstants;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Hardware;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.ScoringSystem;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.SystemData;
import org.firstinspires.ftc.teamcode.OpModes.ExoMode;
import org.firstinspires.ftc.teamcode.CustomPathing.AutoDrive;
import org.firstinspires.ftc.teamcode.CustomPathing.Math.Geometry.Pose;

@Autonomous(group = "red", preselectTeleOp = "😈🔥")
public class RedFar_SortingCompatible extends ExoMode {
    private Hardware hardware;
    private SwerveDrive swerve;
    private ScoringSystem system;
    private AutoDrive auto;

    @Override
    protected void Init() {
        new SystemData()
                .add(SystemConstants.OpMode.AUTONOMOUS)
                .setVelocityTimeout(true)
                .setAutoOnBlue(false);

        hardware = Hardware.getInstance(this);
        swerve = new SwerveDrive(this);
        system = new ScoringSystem(this);

        auto = new AutoDrive(this, swerve, system, startPoseRedFar);

        system.indexer.preload();
        system.setTransferArm(false);

    }

    @Override
    protected void WhenStarted() {
        preload();
        gateCycle1();
        leave();

        system.shooter.off();
    }

    private void preload() {
        auto.driveTo(new Pose(-140, -40, Math.toRadians(337.5)), 0.8, 2000)
                .lockHeadingToGoal(true)
                .moveSystem(() -> system.shooter.on())
                .waitDrive(0.96, false)
                .waitAction(() -> system.shooter.ready())
                .waitMs(1200)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.isShooting)
                .moveSystem(() -> system.setTransferArm(true))
                .lockHeadingToGoal(false);
    }

    private void gateCycle1() {
        auto.driveTo(new Pose(-140, -154, Math.toRadians(-90)), 0.8, 3000)
                .moveSystem(() -> system.intake.on())
                .waitDriveActionFailSafe(0.98, false, () -> system.isIndexerFull)

                .driveTo(new Pose(-164, -156, Math.toRadians(-90)), 0.6, 500)
                .waitDriveActionFailSafe(0.98, false, () -> system.isIndexerFull)
                .driveTo(new Pose(-140, -156, Math.toRadians(-90)), 0.6, 500)
                .waitDriveActionFailSafe(0.98, false, () -> system.isIndexerFull)
                .driveTo(new Pose(-164, -156, Math.toRadians(-90)), 0.6, 500)
                .waitDriveActionFailSafe(0.98, false, () -> system.isIndexerFull)

                .driveTo(new Pose(-140, -40, Math.toRadians(338)), 0.9, 4000)
                .waitMs(400)
                .moveSystem(() -> {
                    new Thread(() -> system.load()).start();

                    try{ Thread.sleep(200); } catch (InterruptedException e) {}
                    system.intake.reverse();
                    try{ Thread.sleep(400); } catch (InterruptedException e) {}
                    system.intake.off();
                })
                .waitDrive(0.4)
                .lockHeadingToGoal(true)
                .waitDrive(0.96)

                .waitAction(() -> system.shooter.ready())
                .waitMs(850)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.isShooting)
                .moveSystem(() -> system.setTransferArm(true))
                .lockHeadingToGoal(false);
    }

    private void leave() {
        auto.driveTo(new Pose(-155, -100, Math.toRadians(0)), 0.8)
                .waitDrive(0.97);
    }



    @Override
    protected void Loop() {}
}