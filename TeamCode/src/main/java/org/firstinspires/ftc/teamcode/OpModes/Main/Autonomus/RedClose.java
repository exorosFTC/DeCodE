package org.firstinspires.ftc.teamcode.OpModes.Main.Autonomus;

import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoAngularD;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoAngularP;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoLinearD;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoLinearP;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.startPose;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.startPoseRedClose;
import static org.firstinspires.ftc.teamcode.Pathing.Math.MathFormulas.midPoint;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandBase.Constants.Enums;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Hardware;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.ScoringSystem;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.SystemData;
import org.firstinspires.ftc.teamcode.OpModes.ExoMode;
import org.firstinspires.ftc.teamcode.Pathing.AutoDrive;
import org.firstinspires.ftc.teamcode.Pathing.Math.Point;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;
import org.firstinspires.ftc.teamcode.Pathing.PurePursuitController;

@Config
@Autonomous(name = "RedClose", group = "main")
public class RedClose extends ExoMode {
    private Hardware hardware;
    private SwerveDrive swerve;
    private ScoringSystem system;
    private AutoDrive auto;

    public static double
            r1 = 5;
    public static double
            l1 = 15;

    public static double angularP = AutoAngularP, angularD = AutoAngularD;
    public static double linearP = AutoLinearP, linearD = AutoLinearD;

    public static Pose
            path1_point1 = new Pose(30, -45, Math.toRadians(315));

    @Override
    protected void Init() {
        hardware = Hardware.getInstance(this);
        swerve = new SwerveDrive(this);
        system = new ScoringSystem(this);

        auto = new AutoDrive(this, swerve, system, startPoseRedClose);

        hardware.limelight.start();
        hardware.limelight.setPipeline(Enums.Pipeline.RANDOMIZATION);

        system.indexer.preload();

        new SystemData()
                .add(Enums.OpMode.AUTONOMUS)
                .setAutoOnBlue(false)
                .getLoopTime(true);


        while (opModeInInit()) {
            auto.linearC.setPID(linearP, 0, linearD);
            auto.angularC.setPID(angularP, 0, angularD);
        }
    }

    @Override
    protected void WhenStarted() {
        auto.drivePath(new PurePursuitController()
                    .setMode(Enums.HeadingMode.LERP)
                    .setStopRadius(r1)
                    .setLookahead(l1)
                 .addPoint(midPoint(startPose, path1_point1, 0))
                 .addPoint(path1_point1))
             .waitAction(() -> auto.controller.reachedSegment(2),
                  () -> hardware.limelight.getRandomization())
             .moveSystem(() -> {
                  hardware.limelight.stop();
                  system.indexer.indexPattern();
                  system.shooter.on();
             })
             .waitAction(() -> !auto.controller.isBusy())
             .moveSystem(() -> system.shootSequence());

    }

    @Override
    protected void Loop() {}
}
