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
        firstLine();
        thirdLine();
        leave();

        auto.end();
    }

    private void preload() {
        auto.driveTo(new Pose(75, -60, Math.toRadians(340)), 30)
                .moveSystem(() -> system.indexer.home())
                .waitDrive(() -> hardware.limelight.getRandomization(), 0.7, true)
                .moveSystem(() -> {
                    system.shooter.on();
                })
                .driveTo(new Pose(75, -60, Math.toRadians(340)), 30)
                .moveSystem(() -> swerve.lockHeadingToGoal(true))
                .waitMs(1000)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.indexer.isBusy())
                .moveSystem(() -> swerve.lockHeadingToGoal(false));
    }

    private void firstLine() {
        auto.driveTo(new Pose(35, -60, Math.toRadians(270)), 30)
                .moveSystem(() -> system.indexer.home())
                .waitDrive(0.85)
                .driveTo(new Pose(35, -129, Math.toRadians(270)), 30)
                .moveSystem(() -> system.intake.on())
                .waitDrive(0.88)
                .driveTo(new Pose(84, -60, Math.toRadians(340)), 30, 2000)
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
                .waitMs(1000)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.indexer.isBusy())
                .moveSystem(() -> swerve.lockHeadingToGoal(false));
    }

    private void secondLine() {
        auto.driveTo(new Pose(-15, -75, Math.toRadians(270)), 30)
                //.moveSystem(() -> system.indexer.home())
                .waitDrive(0.9)
                .moveSystem(() -> system.intake.on())
                .driveTo(new Pose(-15, -156, Math.toRadians(270)), 30)
                .waitDrive(0.87)
                .driveTo(new Pose(-15, -60, Math.toRadians(270)), 30, 2000)
                .moveSystem(() -> {
                    system.intake.reverse();
                    try{ Thread.sleep(400); } catch (InterruptedException e) {}
                    system.intake.off();
                })
                .waitDrive(0.7)
                .driveTo(new Pose(85, -60, Math.toRadians(340)), 30)
                .waitDrive(0.7)
                .moveSystem(() -> {
                    //system.indexer.indexPattern();
                    system.shooter.on();
                })
                .waitDrive(0.9)
                .moveSystem(() -> swerve.lockHeadingToGoal(true))
                .waitMs(1000)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.indexer.isBusy())
                .moveSystem(() -> swerve.lockHeadingToGoal(false));
    }

    private void thirdLine() {
        auto.driveTo(new Pose(-73, -60, Math.toRadians(270)), 30)
                .waitDrive(0.95)
                .moveSystem(() -> system.intake.on())
                .driveTo(new Pose(-83, -156, Math.toRadians(270)), 30)
                .waitDrive(0.87)
                .driveTo(new Pose(-83, -60, Math.toRadians(270)), 30, 2000)
                .moveSystem(() -> {
                    system.intake.reverse();
                    try{ Thread.sleep(400); } catch (InterruptedException e) {}
                    system.intake.off();
                })
                .waitDrive(0.7)
                .driveTo(new Pose(135, -60, Math.toRadians(315)), 30)
                .waitDrive(0.8)
                .moveSystem(() -> {
                    //system.indexer.indexPattern();
                    system.shooter.on();
                })
                .waitDrive(0.9)
                .moveSystem(() -> swerve.lockHeadingToGoal(true))
                .waitMs(1000)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.indexer.isBusy())
                .moveSystem(() -> swerve.lockHeadingToGoal(false));
    }

    private void leave() {
        auto.driveTo(new Pose(135, -60, Math.toRadians(-75)), 30)
                .waitDrive(0.9);
    }



    @Override
    protected void Loop() {}

}
