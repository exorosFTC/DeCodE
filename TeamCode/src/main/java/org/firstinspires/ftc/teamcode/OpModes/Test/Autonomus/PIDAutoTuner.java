package org.firstinspires.ftc.teamcode.OpModes.Test.Autonomus;

import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AngularD;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AngularP;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.LinearD;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.LinearP;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.CommandBase.Constants.Enums;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.SystemData;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Hardware;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.ScoringSystem;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.OpModes.ExoMode;
import org.firstinspires.ftc.teamcode.Pathing.AutoDrive;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

@Config
@Autonomous(name = "MovementPIDTuner", group = "tuning")
public class PIDAutoTuner extends ExoMode {
    private Hardware hardware;
    private SwerveDrive swerve;
    private ScoringSystem system;
    private AutoDrive auto;

    public static double linP = LinearP, angP = AngularP;
    public static double linD = LinearD, angD = AngularD;
    public static double x, y, head;

    public GamepadEx g2;

    @Override
    protected void Init() {
        hardware = Hardware.getInstance(this);
        swerve = new SwerveDrive(this);
        system = new ScoringSystem(this);

        new SystemData()
                        .add(Enums.OpMode.AUTONOMUS)
                        .setAutoOnBlue(false)
                        .getLoopTime(true);

        g2 = new GamepadEx(gamepad2);
    }

    @Override
    protected void WhenStarted() {
        auto = new AutoDrive(this, swerve, system, new Pose());
    }

    @Override
    protected void Loop() {
        if (g2.wasJustPressed(GamepadKeys.Button.B))
            auto.driveTo(new Pose(x, y, Math.toRadians(head)));

        g2.readButtons();
        auto.linearC.setPID(linP, 0, linD);
        auto.angularC.setPID(angP, 0, angD);
    }
}
