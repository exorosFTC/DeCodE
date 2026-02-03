package org.firstinspires.ftc.teamcode.OpModes.Test.Autonomus;

import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AUTO_STRAFING_SLEW_RATE_LIMIT;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AUTO_TURNING_SLEW_RATE_LIMIT;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoAngularD;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoAngularP;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoAngularVelocityMultiplier;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoLinearDx;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoLinearPx;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoLinearVelocityMultiplier;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoSwerveModuleD;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoSwerveModuleP;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandBase.Constants.SystemConstants;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.SystemData;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Hardware;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.ScoringSystem;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.OpModes.ExoMode;
import org.firstinspires.ftc.teamcode.CustomPathing.AutoDrive;
import org.firstinspires.ftc.teamcode.CustomPathing.Math.Geometry.Pose;

@Config
@Autonomous(name = "MovementPIDTuner", group = "tuning")
public class PIDAutoTuner extends ExoMode {
    private SwerveDrive swerve;
    private ScoringSystem system;
    private AutoDrive auto;

    public static double linP = AutoLinearPx;
    public static double linD = AutoLinearDx;
    public static double moduleP = AutoSwerveModuleP, moduleD = AutoSwerveModuleD;
    public static double angularMultiplier = AutoAngularVelocityMultiplier;
    public static double linearMultiplier = AutoAngularVelocityMultiplier;
    public static double x, y, head, startRadius = 30, targetRadius = 30;

    public GamepadEx g1;

    @Override
    protected void Init() {
        new SystemData()
                .add(SystemConstants.OpMode.AUTONOMOUS)
                .setAutoOnBlue(false);

        swerve = new SwerveDrive(this);
        system = new ScoringSystem(this);

        g1 = new GamepadEx(gamepad1);
    }

    @Override
    protected void WhenStarted() {
        auto = new AutoDrive(this, swerve, system, new Pose());
    }

    @Override
    protected void Loop() {
        if (g1.wasJustPressed(GamepadKeys.Button.B))
            auto.driveTo(new Pose(x, y, Math.toRadians(head)), startRadius, targetRadius);

        g1.readButtons();
        auto.linearCx.setPID(linP, 0, linD);
        auto.linearCy.setPID(linP, 0, linD);

        AutoAngularVelocityMultiplier = angularMultiplier;
        AutoLinearVelocityMultiplier = linearMultiplier;

        swerve.setModulePID(moduleP, 0, moduleD);
    }
}
