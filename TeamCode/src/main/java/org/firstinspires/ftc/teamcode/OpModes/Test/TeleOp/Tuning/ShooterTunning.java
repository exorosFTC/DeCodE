package org.firstinspires.ftc.teamcode.OpModes.Test.TeleOp.Tuning;

import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.blueGoalPosition;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.HardwareNames.ShooterMotor1;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.HardwareNames.ShooterMotor2;
import static org.firstinspires.ftc.teamcode.Hardware.Robot.Scoring.Subsystems.Shooter.MAX_RPS;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.Constants.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Drivetrain.Swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Scoring.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

@Config
@TeleOp(name = "Shooter", group = "main")
public class ShooterTunning extends LinearOpMode {
    private Hardware hardware;
    private SwerveDrive swerve;
    private Shooter shooter;

    private GamepadEx g1;

    public static double kS, kV, kP, kI, kD;
    public static double power, angle;

    public boolean lockHeading = false;

    @Override
    public void runOpMode() throws InterruptedException {
        hardware = Hardware.getInstance(this);
        swerve = new SwerveDrive(this);
        shooter = new Shooter(this);

        g1 = new GamepadEx(gamepad1);

        kP = shooter.kP;
        kI = shooter.kI;
        kD = shooter.kD;

        shooter.on = true;

        waitForStart();

        new Thread(() -> {
            while (opModeIsActive()) {
                swerve.update(
                        new Pose(-g1.getLeftY(),
                                g1.getLeftX(),
                                g1.getRightX() * 0.5));

                if (g1.wasJustPressed(GamepadKeys.Button.B)) {
                    lockHeading = !lockHeading;
                    swerve.lockHeadingToGoal(lockHeading);
                }

                g1.readButtons();
                hardware.bulk.clearCache(Enums.Hubs.ALL);
            }
        }).start();

        while (opModeIsActive()) {
            double distance = swerve.localizer.getRobotPosition().distanceTo(blueGoalPosition);

            shooter.kP = kP;
            shooter.kI = kI;
            shooter.kD = kD;

            shooter.targetPower = power;
            shooter.targetAngle = angle;

            swerve.localizer.update();
            shooter.update();

            hardware.telemetry.addData("distance", distance);
            hardware.telemetry.addData("velocity targert", shooter.targetPower * MAX_RPS);
            hardware.telemetry.addData("velocity", hardware.motors.get(ShooterMotor2).getVelocity(AngleUnit.DEGREES));
            hardware.telemetry.addData("ready", shooter.ready());
            hardware.bulk.clearCache(Enums.Hubs.ALL);
            hardware.telemetry.update();
        }
    }
}
