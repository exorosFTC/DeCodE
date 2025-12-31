package org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Hardware;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.SystemBase;

public class Intake extends SystemBase {
    private final Hardware hardware;
    private final LinearOpMode opMode;

    public double intakeVoltage = 0;
    public boolean reversed = false;

    public Intake(LinearOpMode opMode) {
        this.hardware = Hardware.getInstance(opMode);
        this.opMode = opMode;
    }


    @Override
    public void read() {
        if (!on) return;

        intakeVoltage = hardware.IntakeMotor.getCurrent(CurrentUnit.AMPS);
    }

    @Override
    public void write() {}

    @Override
    public void on() {
        super.on();
        reversed = false;
        hardware.IntakeMotor.setPower(-1);
    }

    public void reverse() {
        super.on();
        reversed = true;
        hardware.IntakeMotor.setPower(1);
    }

    @Override
    public void off() {
        super.off();
        hardware.IntakeMotor.setMotorDisable();
    }
}
