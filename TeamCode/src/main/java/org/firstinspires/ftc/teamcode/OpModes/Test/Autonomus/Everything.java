package org.firstinspires.ftc.teamcode.OpModes.Test.Autonomus;

import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.POSE;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Hardware.Constants.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Data;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Drivetrain.Swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.OpModes.ExoMode;
import org.firstinspires.ftc.teamcode.Pathing.AutoDrive;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

@Config
@Autonomous(name = "ValuesAutoTuner", group = "tuning")
public class Everything extends ExoMode {
    private AutoDrive auto;
    private SwerveDrive swerve;

    private GamepadEx g1;

    public static double x = 0, y = 0, head = 0;

    @Override
    protected void Init() {
        new Data()
                        .add(Enums.OpMode.AUTONOMUS)
                        .setAutoOnBlue(false)
                        .getLoopTime(true)
                        .setUsingOpenCv(false)
                        .setUsingAprilTag(false)
                        .setUsingAcceleration(false)
                        .setUsingExponentialInput(false);

        swerve = new SwerveDrive(this);
        g1 = new GamepadEx(gamepad1);
    }

    @Override
    protected void WhenStarted() {
        auto = new AutoDrive(this, swerve, new Pose())
                .pause()
                .disableWheelMotors();
    }

    @Override
    protected void Loop() {
        Pose position = auto.getPosition();
        g1.readButtons();

        //if (g1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER))
        //    auto.setPose(new Pose(x, y, Math.toRadians(head)));
    }
}
