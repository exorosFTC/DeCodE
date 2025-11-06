package org.firstinspires.ftc.teamcode.OpModes.Test.TeleOp.Tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Robot.Drivetrain.Swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Scoring.Subsystems.Shooter;

@Config
@TeleOp(name = "Shooter", group = "main")
public class ShooterTunning extends LinearOpMode {
    private Hardware hardware;
    private SwerveDrive swerve;
    private Shooter shooter;

    public static double kS, kV, kP, kI, kD;
    public static double velocity, angle;

    @Override
    public void runOpMode() throws InterruptedException {
        hardware = Hardware.getInstance(this);
        swerve = new SwerveDrive(this);
        shooter = new Shooter(this);

        kS = shooter.kS;
        kV = shooter.kV;
        kP = shooter.kP;
        kI = shooter.kI;
        kD = shooter.kD;

        waitForStart();

        while (opModeIsActive()) {
            shooter.kS = kS;
            shooter.kV = kV;
            shooter.kP = kP;
            shooter.kI = kI;
            shooter.kD = kD;

            shooter.targetVelocity = velocity;
            shooter.targetAngle = angle;

            shooter.update();

            //hardware.telemetry.addData("distance: ");
        }
    }
}
