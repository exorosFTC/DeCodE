package org.firstinspires.ftc.teamcode.OpModes.Main.Autonomus;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.startPoseRedFar;

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

@Autonomous(group = "main", preselectTeleOp = "😈🔥")
public class RedFar_9 extends ExoMode {
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

        hardware.limelight.start();
        hardware.limelight.setPipeline(LimelightEx.Pipeline.RANDOMIZATION);

        system.indexer.preload();
    }



    @Override
    protected void WaitForStart(){
        hardware.limelight.getRandomization();
    }

    @Override
    protected void WhenStarted() {
        hardware.limelight.stop();

        preload();
        thirdLine();
        humanPlayerLine();
        leave();
    }

    private void preload() {
        auto.driveTo(new Pose(-140, -28, Math.toRadians(335.5)), 0.8, 2000)
                .moveSystem(() -> system.shooter.on())
                .waitDrive(0.95, true)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.indexer.isBusy());
    }

    private void thirdLine() {
        auto.driveTo(new Pose(-90, -70, Math.toRadians(270)), 0.6, 4000)
                .waitDrive(0.8)
                .moveSystem(() -> system.intake.on())
                .driveTo(new Pose(-90, -170, Math.toRadians(270)), 0.4, 5000)
                .waitDrive(0.6) //0.9
                .waitMs(400)
                .driveTo(new Pose(-140, -28, Math.toRadians(335)), 0.7, 5000)
                .moveSystem(() -> {
                    system.intake.reverse();
                    try{ Thread.sleep(400); } catch (InterruptedException e) {}
                    system.intake.off();
                })
                .waitDrive(0.6)
                .moveSystem(() -> system.shooter.on())
                .waitDrive(0.95, true)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.indexer.isBusy());
    }

    private void humanPlayerLine() {
        auto.driveTo(new Pose(-150, -164, Math.toRadians(-90)), 0.6, 3000)
                .moveSystem(() -> system.intake.on())

                .waitDrive(0.94)
                .driveTo(new Pose(-150, -100, Math.toRadians(-90)), 0.9, 1000)
                .waitDrive(0.004)
                .driveTo(new Pose(-150, -164, Math.toRadians(-90)), 0.9, 1000)
                .waitDrive(0.33)
                .driveTo(new Pose(-150, -100, Math.toRadians(-90)), 0.9, 1000)
                .waitDrive(0.004)
                .driveTo(new Pose(-150, -164, Math.toRadians(-90)), 0.9, 1000)
                .waitDrive(0.33)
                .waitMs(500)

                .driveTo(new Pose(-140, -28, Math.toRadians(335)), 0.7, 5000)
                .moveSystem(() -> {
                    system.intake.reverse();
                    try{ Thread.sleep(400); } catch (InterruptedException e) {}
                    system.intake.off();
                })
                .waitDrive(0.5)
                .moveSystem(() -> system.shooter.on())
                .waitDrive(0.95, true)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.indexer.isBusy());
    }

    private void leave() {
        auto.driveTo(new Pose(-150, -108, Math.toRadians(0)), 0.8)
                .waitDrive(0.9);
    }



    @Override
    protected void Loop() {}
}