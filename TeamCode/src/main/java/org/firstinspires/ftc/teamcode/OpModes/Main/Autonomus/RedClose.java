package org.firstinspires.ftc.teamcode.OpModes.Main.Autonomus;

import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoAngularD;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoAngularP;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoLinearDx;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoLinearPx;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoLinearPy;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.startPoseRedClose;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandBase.Constants.Enums;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Hardware;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.ScoringSystem;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.SystemData;
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

    public static double linearThreshold = 0.08;
    public static double angularThreshold = 7;

    @Override
    protected void Init() {
        new SystemData()
                .add(Enums.OpMode.AUTONOMUS)
                .setAutoOnBlue(false)
                .getLoopTime(true);

        hardware = Hardware.getInstance(this);
        swerve = new SwerveDrive(this);
        system = new ScoringSystem(this);

        auto = new AutoDrive(this, swerve, system, startPoseRedClose);

        hardware.limelight.start();
        hardware.limelight.setPipeline(Enums.Pipeline.RANDOMIZATION);

        system.indexer.preload();

        while (opModeInInit()) {
            auto.linearCx.setPID(linearPx, 0, linearDx);
            auto.linearCy.setPID(linearPy, 0, linearDy);
            auto.angularC.setPID(angularP, 0, angularD);

            auto.setBusyThresholdLinear(linearThreshold);
            auto.setBusyThresholdAngular(Math.toRadians(angularThreshold));
        }
    }

    @Override
    protected void WhenStarted() {
        auto.driveTo(new Pose(75, -60, Math.toRadians(45)))
             // cycle 1 + read randomization
             .moveSystem(() -> system.indexer.home())
                .waitDrive(() -> hardware.limelight.getRandomization(), 4)
             .moveSystem(() -> {
                  hardware.limelight.stop();
                  //system.indexer.indexPattern();
                  system.shooter.on();
             })
             .driveTo(new Pose(75, -60, Math.toRadians(315)))
                .waitDrive(1)
             .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.indexer.isBusy())

             // cycle 2
             .driveTo(new Pose(35, -60, Math.toRadians(270)))
             .moveSystem(() -> system.indexer.home())
                .waitDrive(2)
             .moveSystem(() -> {
                system.intake.on();
                system.isIntakeEnabled = true;
                system.indexer.on();
             })
             .driveTo(new Pose(35, -127, Math.toRadians(270)), 1200)
                .waitDrive(1)
                .waitMs(300)
             .driveTo(new Pose(75, -60, Math.toRadians(315)))
             .moveSystem(() -> {
                 system.intake.reverse();
                 try{ Thread.sleep(400); } catch (InterruptedException e) {}
                 system.intake.off();
                 system.isIntakeEnabled = false;
                 system.shooter.on();

             })
                .waitDrive(1)
             .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.indexer.isBusy())

                // cycle 3
             .driveTo(new Pose(-25, -60, Math.toRadians(270)))
             .moveSystem(() -> system.indexer.home())
                .waitDrive(2)
             .moveSystem(() -> {
                    system.intake.on();
                    system.isIntakeEnabled = true;
                    system.indexer.on();
                })
             .driveTo(new Pose(-27, -149, Math.toRadians(270)), 1200)
                .waitDrive(1)
                .waitMs(300)
             .driveTo(new Pose(-27, -60, Math.toRadians(270)))
             .moveSystem(() -> {
                    system.intake.reverse();
                    try{ Thread.sleep(400); } catch (InterruptedException e) {}
                    system.intake.off();
                    system.isIntakeEnabled = false;
                    system.shooter.on();
                })
                .waitDrive(1.5)
             .driveTo(new Pose(77, -60, Math.toRadians(315)))
                .waitAction(() -> !system.indexer.isIndexing)
                .waitDrive(1)
             .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.indexer.isBusy())
             .moveSystem(() -> system.indexer.home())


                // cycle 4
             .driveTo(new Pose(-75, -60, Math.toRadians(270)))
             .moveSystem(() -> system.indexer.home())
                .waitDrive(2)
                .moveSystem(() -> {
                    system.intake.on();
                    system.isIntakeEnabled = true;
                    system.indexer.on();
                })
                .driveTo(new Pose(-75, -149, Math.toRadians(270)), 1200)
                .waitDrive(1)
                .waitMs(300)
                .driveTo(new Pose(77, -60, Math.toRadians(315)))
                .moveSystem(() -> {
                    system.intake.reverse();
                    try{ Thread.sleep(400); } catch (InterruptedException e) {}
                    system.intake.off();
                    system.isIntakeEnabled = false;
                    system.shooter.on();
                })
                .waitDrive(1.5)
                .waitAction(() -> !system.indexer.isIndexing)
                .waitDrive(1)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.indexer.isBusy())

                // park
                .driveTo(new Pose(-20, -80, Math.toRadians(270)))
                .waitDrive()
                .end();


    }

    @Override
    protected void Loop() {}


    private void preload() {
        auto.driveTo(new Pose(75, -60, Math.toRadians(45)))
                // cycle 1 + read randomization
                .moveSystem(() -> system.indexer.home())
                .waitDrive(() -> hardware.limelight.getRandomization(), 4)
                .moveSystem(() -> {
                    hardware.limelight.stop();
                    system.shooter.on();
                })
                .driveTo(new Pose(75, -60, Math.toRadians(315)))
                .waitDrive(1)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.indexer.isBusy());
    }

    private void cycle1() {}

    private void cycle2() {}

    private void cycle3() {}

    private void park() {}
}
