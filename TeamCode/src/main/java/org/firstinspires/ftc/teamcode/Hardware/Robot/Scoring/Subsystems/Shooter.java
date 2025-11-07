package org.firstinspires.ftc.teamcode.Hardware.Robot.Scoring.Subsystems;

import static androidx.core.math.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.HardwareNames.ShooterMotor1;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.HardwareNames.ShooterMotor2;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Hardware.Robot.Hardware;

public class Shooter {
    private final Hardware hardware;
    private final PIDController controller;

    private double distance;
    public boolean on = false;

    public static final double MAX_RPS = 2400;

    public double targetPower = 0;
    public double targetAngle = 0;

    public double kS = 0.05;
    public double kV = 1.0 / (0.8*MAX_RPS); // scale so targetRPS* kV ≈ needed power; 0.8 accounts for losses

    public double kP = 1.;
    public double kI = 0;
    public double kD = 0.3;


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

    public void setDistance(double distance) { this.distance = distance; }

    public boolean ready() { return Math.abs(targetPower * MAX_RPS - hardware.motors.get(ShooterMotor1).getVelocity()) < 20; }



    public void update() {
        if (!on) return;
        controller.setPID(kP, kI, kD);

        targetPower();
        targetAngle();

        double ff = kS * Math.signum(targetPower) + kV * targetPower;
        double power = clamp(ff + controller.calculate(hardware.motors.get(ShooterMotor1).getVelocity(), targetPower), -1.0, 1.0);


        hardware.motors.get(ShooterMotor1).setPower(power);
        hardware.motors.get(ShooterMotor2).setPower(power);
    }

    //TODO: figure these out
    private void targetPower() {}

    private void targetAngle() {}



    public void on() { this.on = true; }

    public void off() {
        this.on = false;

        hardware.motors.get(ShooterMotor1).setMotorDisable();
        hardware.motors.get(ShooterMotor2).setMotorDisable();
    }
}


