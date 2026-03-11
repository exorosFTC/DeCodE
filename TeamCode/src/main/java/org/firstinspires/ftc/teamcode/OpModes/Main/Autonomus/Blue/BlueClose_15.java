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
        gateCycle();
        gateCycle();
        firstLine();

        system.shooter.off();
    }

    private void preload() {
        auto.driveTo(new Pose(60, 40, Math.toRadians(55)), 0.3)
                .moveSystem(() -> system.shooter.on())
                .lockHeadingToGoal(true)
                .waitDrive(0.8)
                .waitMs(300)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.isShooting)
                .moveSystem(() -> system.setTransferArm(true))
                .lockHeadingToGoal(false);
    }

    private void firstLine() {
        auto.moveSystem(() -> system.intake.on())
                .driveTo(new Pose(40, 50, Math.toRadians(90)), 0.6)
                .waitDrive(0.4)
                .driveTo(new Pose(40, 128, Math.toRadians(90)), 0.5)
                .waitDriveActionFailSafe(0.965, false,  () -> system.isIndexerFull)
                .driveTo(new Pose(90, 40, Math.toRadians(38)), 0.6)
                .moveSystem(() -> {
                    new Thread(() -> system.load()).start();

                    try{ Thread.sleep(200); } catch (InterruptedException e) {}
                    system.intake.reverse();
                    try{ Thread.sleep(400); } catch (InterruptedException e) {}
                    system.intake.off();
                })
                .waitDrive(0.2)
                .lockHeadingToGoal(true)
                .waitDrive(0.97, false)
                .waitMs(300)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.isShooting)
                .moveSystem(() -> system.setTransferArm(true))
                .lockHeadingToGoal(false);
    }

    private void secondLine() {
        auto.moveSystem(() -> system.intake.on())
                .driveTo(new Pose(-20, 42, Math.toRadians(90)), 0.7)
                .waitDrive(0.6)
                .driveTo(new Pose(-20, 156, Math.toRadians(90)), 0.5, 2500)
                .waitDriveActionFailSafe(0.88, false, () -> system.isIndexerFull)
                .driveTo(new Pose(-20, 60, Math.toRadians(90)), 0.7)
                .waitDrive(0.4)
                .moveSystem(() ->
                        new Thread(() -> {
                            new Thread(() -> system.load()).start();

                            try{ Thread.sleep(200); } catch (InterruptedException e) {}
                            system.intake.reverse();
                            try{ Thread.sleep(400); } catch (InterruptedException e) {}
                            system.intake.off();
                        }).start())
                .driveTo(new Pose(60, 60, Math.toRadians(38.5)), 0.6)
                .waitDrive(0.3)
                .lockHeadingToGoal(true)
                .waitDrive(0.97, false)
                .waitMs(300)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.isShooting)
                .moveSystem(() -> system.setTransferArm(true))
                .lockHeadingToGoal(false);
    }

    private void gateCycle() {
        auto.moveSystem(() -> system.intake.on())
                .driveTo(new Pose(-10, 50, Math.toRadians(90)), 0.55)
                .waitDrive(0.34)
                .driveTo(new Pose(-10, 149, Math.toRadians(80)), 0.55)
                .waitDrive(0.65)

                .driveTo(new Pose(-10, 149, Math.toRadians(80)), 0.3, 400)
                .waitDrive(0.87)
                .driveTo(new Pose(-35, 146, Math.toRadians(50)), 0.5, 1000)
                .waitDriveActionFailSafe(0.8, false, () -> system.isIndexerFull)
                .waitMsActionFailSafe(500, () -> system.isIndexerFull)
                .driveTo(new Pose(-23, 146, Math.toRadians(90)), 0.5, 1000)
                .waitDriveActionFailSafe(0.4, false, () -> system.isIndexerFull)
                .waitMsActionFailSafe(6600, () -> system.isIndexerFull)
                .driveTo(new Pose(-23, 70, Math.toRadians(90)), 0.6)
                .moveSystem(() -> {
                    new Thread(() -> system.load()).start();

                    try{ Thread.sleep(200); } catch (InterruptedException e) {}
                    system.intake.reverse();
                    try{ Thread.sleep(400); } catch (InterruptedException e) {}
                    system.intake.off();
                })
                .lockHeadingToGoal(true)
                .waitDrive(0.4)
                .driveTo(new Pose(60, 60, Math.toRadians(38.5)), 0.7)

                .waitDrive(0.97, false)
                .waitMs(300)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.isShooting)
                .lockHeadingToGoal(false);
    }



    @Override
    protected void Loop() {}
}