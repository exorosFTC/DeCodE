package org.firstinspires.ftc.teamcode.Hardware.Robot.Scoring.Subsystems;

import static org.firstinspires.ftc.teamcode.Hardware.Constants.HardwareNames.IntakeMotor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Hardware.Robot.Hardware;

public class Intake {
    private final Hardware hardware;
    private final LinearOpMode opMode;

    public boolean on = false;

    public Intake(LinearOpMode opMode) {
        this.hardware = Hardware.getInstance(opMode);
        this.opMode = opMode;

        hardware.motors.get(IntakeMotor).setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        hardware.motors.get(IntakeMotor).setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }



    public void on() { on = true; hardware.motors.get(IntakeMotor).setPower(1); }

    public void reverse() { on = true; hardware.motors.get(IntakeMotor).setPower(-1); }

    public void off() { on = false; hardware.motors.get(IntakeMotor).setMotorDisable(); }
}
