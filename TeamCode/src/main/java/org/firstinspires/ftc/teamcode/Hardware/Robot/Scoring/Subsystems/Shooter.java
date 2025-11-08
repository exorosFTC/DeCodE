package org.firstinspires.ftc.teamcode.Hardware.Robot.Scoring.Subsystems;

import static androidx.core.math.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.POSE;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.blueGoalPosition;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.redGoalPosition;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.HardwareNames.ShooterHoodServo;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.HardwareNames.ShooterMotor1;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.HardwareNames.ShooterMotor2;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.SystemConstants.autoOnBlue;
import static org.firstinspires.ftc.teamcode.Pathing.Math.MathFormulas.clip;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Hardware;

public class Shooter {
    private final Hardware hardware;
    private final PIDController controller;

    private double distance;
    public boolean on = false;

    public static final double MAX_RPS = 600;
    public static final double MAX_ANGLE = 1;
    public static final double MIN_ANGLE = 0;

    public double targetPower = 0;
    public double targetAngle = 0;

    private double POWER = 0;

    public double kP = 1e-4;
    public double kI = 1e-8;
    public double kD = 0;


    public Shooter(LinearOpMode opMode) {
        this.hardware = Hardware.getInstance(opMode);
        controller = new PIDController(kP, kI, kD);

        hardware.motors.get(ShooterMotor1).setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        hardware.motors.get(ShooterMotor1).setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        hardware.motors.get(ShooterMotor1).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        hardware.motors.get(ShooterMotor2).setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        hardware.motors.get(ShooterMotor2).setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        hardware.motors.get(ShooterMotor2).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        hardware.motors.get(ShooterMotor2).setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public boolean ready() {
        hardware.telemetry.addData("target RPS", targetPower * MAX_RPS);
        hardware.telemetry.addData("current RPS", hardware.motors.get(ShooterMotor2).getVelocity(AngleUnit.DEGREES));
        hardware.telemetry.addData("ready", Math.abs(targetPower * MAX_RPS - hardware.motors.get(ShooterMotor2).getVelocity(AngleUnit.DEGREES)) < 20);
        hardware.telemetry.update();

        return Math.abs(targetPower * MAX_RPS - hardware.motors.get(ShooterMotor2).getVelocity(AngleUnit.DEGREES)) < 20;
    }



    public void update() {
        if (!on) return;

        controller.setPID(kP, kI, kD);
        distance = POSE.distanceTo(autoOnBlue ? blueGoalPosition : redGoalPosition);

        targetPower();
        targetAngle();

        double pid = controller.calculate(hardware.motors.get(ShooterMotor2).getVelocity(AngleUnit.DEGREES), targetPower * MAX_RPS);
        this.POWER = (POWER + pid) * (13.0 / hardware.batteryVoltageSensor.getVoltage()); // normalize with battery voltage

        hardware.motors.get(ShooterMotor1).setPower(this.POWER);
        hardware.motors.get(ShooterMotor2).setPower(this.POWER);

        hardware.servos.get(ShooterHoodServo).setPosition(targetAngle);
    }

    //TODO: figure these out
    private void targetPower() {
        targetPower = clip(distance / 440, 0, 1);
    }

    private void targetAngle() {
        targetAngle = clamp(distance / 440 * 3, MIN_ANGLE, MAX_ANGLE);
    }



    public void on() { this.on = true; }

    public void off() {
        this.on = false;
        this.targetPower = 0;
        this.POWER = 0;

        hardware.motors.get(ShooterMotor1).setMotorDisable();
        hardware.motors.get(ShooterMotor2).setMotorDisable();
    }
}


