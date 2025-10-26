package org.firstinspires.ftc.teamcode.Hardware.Robot.Scoring.Subsystems;

import static org.firstinspires.ftc.teamcode.Hardware.Constants.HardwareNames.IntakeMotor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Hardware.Robot.Hardware;

public class Intake {
    private final Hardware hardware;
    private final LinearOpMode opMode;

    public static double ON = 1;

    public Intake(LinearOpMode opMode) {
        this.hardware = Hardware.getInstance(opMode);
        this.opMode = opMode;

        hardware.motors.get(IntakeMotor).setDirection(DcMotorSimple.Direction.REVERSE);
        hardware.motors.get(IntakeMotor).setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        hardware.motors.get(IntakeMotor).setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }



    public void on() { hardware.motors.get(IntakeMotor).setPower(ON); }

    public void off() { hardware.motors.get(IntakeMotor).setMotorDisable(); }
}
