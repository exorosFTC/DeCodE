package org.firstinspires.ftc.teamcode.OpModes.Main.Autonomus;

import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoAngularD;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoAngularP;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoLinearDx;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoLinearPx;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoLinearPy;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.goalPosition;
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

    public static double angularP = AutoAngularP, angularD = AutoAngularD;
    public static double linearPx = AutoLinearPx, linearDx = AutoLinearDx;
    public static double linearPy = AutoLinearPy, linearDy = AutoLinearDx;

    public static double linearThreshold = 0.1;
    public static double angularThreshold = 9;

    @Override
    protected void Init() {
        new SystemData()
                .add(SystemConstants.OpMode.AUTONOMUS)
                .setAutoOnBlue(true);

        hardware = Hardware.getInstance(this);
        swerve = new SwerveDrive(this);
        system = new ScoringSystem(this);

        auto = new AutoDrive(this, swerve, system, startPoseBlueFar);

        hardware.limelight.start();
        hardware.limelight.setPipeline(LimelightEx.Pipeline.RANDOMIZATION);

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
        auto.driveTo(new Pose(-170, 100, Math.toRadians(0)))
                .waitDrive(() -> hardware.limelight.getRandomization())
                .moveSystem(() -> hardware.limelight.stop())
                .end();
    }

    @Override
    protected void Loop() {}
}
