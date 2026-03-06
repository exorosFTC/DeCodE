package org.firstinspires.ftc.teamcode.OpModes.Main.Autonomus;

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
@Autonomous(group = "main", preselectTeleOp = "😈🔥")
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
        secondLine();
        gateCycle();
        gateCycle();
        firstLine();
    }

    private void preload() {
        auto.driveTo(new Pose(60, -40, Math.toRadians(-55)), 0.35)
                .moveSystem(() -> { system.shooter.on(); })
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.shooter.on)
                .moveSystem(() -> { system.shooter.on(); })
                .waitDrive(0.8);
    }

    private void firstLine() {
        auto.moveSystem(() -> system.intake.on())
                .driveTo(new Pose(40, -50, Math.toRadians(270)), 0.6)
                .waitDrive(0.4)
                .driveTo(new Pose(40, -128, Math.toRadians(270)), 0.5)
                .waitDrive(0.965, false)
                .driveTo(new Pose(90, -40, Math.toRadians(-52)), 0.5)
                .moveSystem(() -> {
                    system.intake.reverse();
                    try{ Thread.sleep(400); } catch (InterruptedException e) {}
                    system.intake.off();
                    system.load();
                })
                .waitDrive(0.5)
                .waitDrive(0.97)
                .waitMs(50)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.shooter.on);
    }

    private void secondLine() {
        auto.moveSystem(() -> system.intake.on())
                .driveTo(new Pose(-23, -50, Math.toRadians(270)), 0.6)
                .waitDrive(0.75)
                .driveTo(new Pose(-23, -156, Math.toRadians(270)), 0.45)
                .waitDriveActionFailSafe(0.812, false, () -> system.isIndexerFull)
                .driveTo(new Pose(-23, -60, Math.toRadians(270)), 0.65)
                .moveSystem(() -> {
                    if (!system.isIndexerFull) return;

                    system.intake.reverse();
                    try{ Thread.sleep(400); } catch (InterruptedException e) {}
                    system.intake.off();
                    system.load();
                })
                .waitDrive(0.4)

                .driveTo(new Pose(60, -60, Math.toRadians(-40.5)), 0.6)
                .waitDrive(0.5)
                .moveSystem(() -> {
                    if (system.isIndexerFull) return;

                    system.intake.reverse();
                    try{ Thread.sleep(400); } catch (InterruptedException e) {}
                    system.intake.off();
                    system.load();
                })
                .waitDrive(0.93, false)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.shooter.on)
                .moveSystem(() -> { system.shooter.on(); });
    }

    private void gateCycle() {
        auto.moveSystem(() -> system.intake.on())
                .driveTo(new Pose(-10, -50, Math.toRadians(270)), 0.55)
                .waitDrive(0.34)
                .driveTo(new Pose(-10, -149, Math.toRadians(-70)), 0.55)
                .waitDrive(0.65)

                .driveTo(new Pose(-10, -149, Math.toRadians(-70)), 0.3, 500)
                .waitDrive(0.88)
                .waitMsActionFailSafe(100, () -> system.isIndexerFull)
                .driveTo(new Pose(-35, -146, Math.toRadians(-50)), 0.5)
                .waitDriveActionFailSafe(0.8, false, () -> system.isIndexerFull)
                .waitMsActionFailSafe(800, () -> system.isIndexerFull)
                .driveTo(new Pose(-23, -149, Math.toRadians(-90)), 0.5)
                .waitDriveActionFailSafe(0.4, false, () -> system.isIndexerFull)
                .waitMsActionFailSafe(400, () -> system.isIndexerFull)

                .driveTo(new Pose(-23, -70, Math.toRadians(270)), 0.6)
                .moveSystem(() -> {
                    system.intake.reverse();
                    try{ Thread.sleep(400); } catch (InterruptedException e) {}
                    system.intake.off();
                    system.load();
                })
                .waitDrive(0.4)
                .driveTo(new Pose(60, -60, Math.toRadians(-41.5)), 0.7)

                .waitDrive(0.95, false)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.shooter.on)
                .moveSystem(() -> { system.shooter.on(); });

    }

    @Override
    protected void Loop() {}

}
