package org.firstinspires.ftc.teamcode.OpModes.Main.Autonomus;

import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoAngularD;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoAngularP;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoLinearDx;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoLinearPx;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoLinearPy;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.startPoseBlueFar;

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
@Autonomous(name = "BlueFar", group = "main", preselectTeleOp = "😈🔥")
public class BlueFar extends ExoMode {
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

        auto = new AutoDrive(this, swerve, system, startPoseBlueFar);

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
        //thirdLine();
        //humanPlayerLine();
        leave();

        auto.end();
    }

    private void preload() {
        auto.driveTo(new Pose(-166, 28, Math.toRadians(25)), 30)
                .moveSystem(() -> {
                    //system.indexer.indexPattern();
                    system.shooter.on();
                })
                .waitDrive(0.61)
                .moveSystem(() -> swerve.lockHeadingToGoal(true))
                .waitMs(600)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.indexer.isBusy())
                .moveSystem(() -> swerve.lockHeadingToGoal(false));
    }

    private void thirdLine() {
        auto.driveTo(new Pose(-115, 88, Math.toRadians(90)), 30, 3000)
                .waitDrive(0.87)
                .moveSystem(() -> system.intake.on())
                .driveTo(new Pose(-115, 170, Math.toRadians(90)), 30, 3000)
                .waitDrive(0.87)
                .waitMs(500)
                .driveTo(new Pose(-166, 22, Math.toRadians(26.5)), 30)
                .moveSystem(() -> {
                    system.intake.reverse();
                    try{ Thread.sleep(400); } catch (InterruptedException e) {}
                    system.intake.off();
                })
                .waitDrive(0.5)
                .moveSystem(() -> {
                    //system.indexer.indexPattern();
                    system.shooter.on();
                })
                .waitDrive(0.91)
                .moveSystem(() -> swerve.lockHeadingToGoal(true))
                .waitMs(600)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.indexer.isBusy())
                .moveSystem(() -> swerve.lockHeadingToGoal(false));
    }

    private void humanPlayerLine() {
        auto.driveTo(new Pose(-126, 176, Math.toRadians(180)), 30, 3500)
                .waitDrive(0.88)
                .moveSystem(() -> system.intake.on())
                .driveTo(new Pose(-190, 176, Math.toRadians(180)), 30, 2500)
                .waitDrive(0.9)
                .driveTo(new Pose(-166, 25, Math.toRadians(24)), 30)
                .moveSystem(() -> {
                    system.intake.reverse();
                    try{ Thread.sleep(400); } catch (InterruptedException e) {}
                    system.intake.off();
                })
                .waitDrive(0.5)
                .moveSystem(() -> {
                    //system.indexer.indexPattern();
                    system.shooter.on();
                })
                .waitDrive(0.91)
                .moveSystem(() -> swerve.lockHeadingToGoal(true))
                .waitMs(600)
                .moveSystem(() -> system.shootSequence())
                .waitAction(() -> !system.indexer.isBusy())
                .moveSystem(() -> swerve.lockHeadingToGoal(false));

    }

    private void leave() {
        auto.driveTo(new Pose(-166, 108, Math.toRadians(0)), 30)
                .waitDrive(0.9);
    }




    @Override
    protected void Loop() {}
}
