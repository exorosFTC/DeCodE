package org.firstinspires.ftc.teamcode.OpModes.Main.Autonomus.Red;

import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoAngularD;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoAngularP;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoAngularVelocityMultiplier;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoLinearDx;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoLinearPx;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoLinearPy;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.TeleOpVelocityMultiplier;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.startPoseRedClose;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandBase.Constants.SystemConstants;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Hardware;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.ScoringSystem;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.SystemData;
import org.firstinspires.ftc.teamcode.OpModes.ExoMode;
import org.firstinspires.ftc.teamcode.CustomPathing.AutoDrive;
import org.firstinspires.ftc.teamcode.CustomPathing.Math.Geometry.Pose;

@Config
@Autonomous(group = "red", preselectTeleOp = "😈🔥")
public class RedClose_15 extends ExoMode {
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
                .setVelocityTimeout(true)
                .setAutoOnBlue(false);

        hardware = Hardware.getInstance(this);
        swerve = new SwerveDrive(this);
        system = new ScoringSystem(this);

        auto = new AutoDrive(this, swerve, system, startPoseRedClose);

        system.indexer.preload();
        system.setTransferArm(false);
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
        secondLine();
        gateCycle();
        gateCycle();
        firstLine();

        auto.lockHeadingToGoal(true);
        system.shooter.off();
    }

    private void preload() {
        auto.driveTo(new Pose(80, -60, Math.toRadians(-42)), 0.5)
                .moveSystem(() -> system.shooter.on())
                .lockHeadingToGoal(true)
                .waitDrive(0.98)
                .waitMs(400)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.isShooting)
                .moveSystem(() -> system.setTransferArm(true))
                .lockHeadingToGoal(false);
    }

    private void firstLine() {
        auto.moveSystem(() -> system.intake.on())
                .driveTo(new Pose(40, -50, Math.toRadians(270)), 0.8)
                .waitDrive(0.4)
                .driveTo(new Pose(40, -128, Math.toRadians(270)), 0.5)
                .waitDriveActionFailSafe(0.965, false,  () -> system.isIndexerFull)
                .driveTo(new Pose(90, -40, Math.toRadians(-48)), 0.7)
                .moveSystem(() -> {
                    new Thread(() -> system.load()).start();

                    try{ Thread.sleep(200); } catch (InterruptedException e) {}
                    system.intake.reverse();
                    try{ Thread.sleep(400); } catch (InterruptedException e) {}
                    system.intake.off();
                })
                .waitDrive(0.2)
                .lockHeadingToGoal(true)
                .waitDrive(0.98, false)
                .waitMs(300)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.isShooting)
                .moveSystem(() -> system.setTransferArm(true));
    }

    private void secondLine() {
        auto.moveSystem(() -> system.intake.on())
                .driveTo(new Pose(-20, -42, Math.toRadians(270)), 0.8)
                .waitDrive(0.5)
                .driveTo(new Pose(-20, -156, Math.toRadians(270)), 0.6, 2500)
                .waitDriveActionFailSafe(0.88, false, () -> system.isIndexerFull)
                .driveTo(new Pose(-20, -60, Math.toRadians(270)), 0.8)
                .waitDrive(0.4)
                .moveSystem(() ->
                    new Thread(() -> {
                        new Thread(() -> system.load()).start();

                        try{ Thread.sleep(200); } catch (InterruptedException e) {}
                        system.intake.reverse();
                        try{ Thread.sleep(400); } catch (InterruptedException e) {}
                        system.intake.off();
                    }).start())
                .driveTo(new Pose(60, -60, Math.toRadians(-42.5)), 0.8)
                .waitDrive(0.3)
                .lockHeadingToGoal(true)
                .waitDrive(0.98, false)
                .waitMs(300)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.isShooting)
                .moveSystem(() -> system.setTransferArm(true))
                .lockHeadingToGoal(false);
    }

    private void gateCycle() {
        auto.moveSystem(() -> system.intake.on())
                .driveTo(new Pose(-10, -50, Math.toRadians(270)), 0.8)
                .waitDrive(0.6)
                .driveTo(new Pose(-10, -146, Math.toRadians(270)), 0.55)
                .waitDrive(0.9)
                .driveTo(new Pose(-10, -142, Math.toRadians(270)), 0.3, 400)
                .waitDrive(0.79)

                .driveTo(new Pose(-45, -142, Math.toRadians(270)), 0.8, 1000)
                .waitDriveActionFailSafe(0.8, false, () -> system.isIndexerFull)
                .driveTo(new Pose(-27, -142, Math.toRadians(270)), 0.8, 1000)
                .waitDriveActionFailSafe(0.8, false, () -> system.isIndexerFull)
                .driveTo(new Pose(-45, -142, Math.toRadians(270)), 0.8, 1000)
                .waitDriveActionFailSafe(0.8, false, () -> system.isIndexerFull)

                .driveTo(new Pose(-23, -70, Math.toRadians(270)), 0.8)
                .moveSystem(() -> {
                    new Thread(() -> system.load()).start();

                    try{ Thread.sleep(200); } catch (InterruptedException e) {}
                    system.intake.reverse();
                    try{ Thread.sleep(400); } catch (InterruptedException e) {}
                    system.intake.off();
                })
                .lockHeadingToGoal(true)
                .waitDrive(0.4)
                .driveTo(new Pose(60, -60, Math.toRadians(38.5)), 0.8)

                .waitDrive(0.98, false)
                .waitMs(300)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.isShooting)
                .lockHeadingToGoal(false);
    }

    @Override
    protected void Loop() {}

}
