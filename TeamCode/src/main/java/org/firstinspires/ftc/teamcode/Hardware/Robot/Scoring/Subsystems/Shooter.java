package org.firstinspires.ftc.teamcode.Hardware.Robot.Scoring.Subsystems;

import static org.firstinspires.ftc.teamcode.Hardware.Constants.HardwareNames.ShooterMotor1;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.HardwareNames.ShooterMotor2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Hardware.Robot.Hardware;

public class Shooter {
    private final Hardware hardware;
    private final LinearOpMode opMode;

    public static final double TICKS_PER_REV = 28 * 20.0;
    public static final double MAX_RPS = 6000.0 / 60.0;

    //feedforward
    private double kS = 0.05;
    private double kV = 1.0 / (0.8*MAX_RPS); // scale so targetRPS* kV ≈ needed power; 0.8 accounts for losses
    private double kA = 0.00;

    //PID (on velocity error)
    private double kP = 0.02;
    private double kI = 0.00;
    private double kD = 0.000;

    //cross-coupling gain
    private double kSync = 0.02;

    //slew limit for target changes (rps per second)
    private double maxTargetSlew = 150.0;



    public Shooter(LinearOpMode opMode) {
        this.hardware = Hardware.getInstance(opMode);
        this.opMode = opMode;

        hardware.motors.get(ShooterMotor1).setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        hardware.motors.get(ShooterMotor1).setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        hardware.motors.get(ShooterMotor1).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        hardware.motors.get(ShooterMotor2).setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        hardware.motors.get(ShooterMotor2).setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        hardware.motors.get(ShooterMotor2).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        hardware.motors.get(ShooterMotor2).setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public void update() {}

    public void off() {
        hardware.motors.get(ShooterMotor1).setMotorDisable();
        hardware.motors.get(ShooterMotor2).setMotorDisable();
    }
}


