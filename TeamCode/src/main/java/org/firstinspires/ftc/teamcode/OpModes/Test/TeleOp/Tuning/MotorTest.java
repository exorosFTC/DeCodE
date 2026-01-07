package org.firstinspires.ftc.teamcode.OpModes.Test.TeleOp.Tuning;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Hardware;

@TeleOp(group = "test")
public class MotorTest extends LinearOpMode {

    private GamepadEx g1;
    private Hardware hardware;

    private DcMotorEx[] motors;
    private int index = 0;

    private double power = 0.0;

    @Override
    public void runOpMode() {
        hardware = Hardware.getInstance(this);
        g1 = new GamepadEx(gamepad1);

        motors = new DcMotorEx[]{
                hardware.LeftFront,
                hardware.LeftBack,
                hardware.RightFront,
                hardware.RightBack,
                hardware.Shooter1,
                hardware.Shooter2,
                hardware.IntakeMotor,
                hardware.IndexerMotor
        };

        for (DcMotorEx m : motors) {
            m.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            m.setPower(0);
        }

        waitForStart();

        while (opModeIsActive()) {
            g1.readButtons();

            if (g1.wasJustPressed(GamepadKeys.Button.B)) {
                motors[index].setPower(0);
                index = (index + 1) % motors.length;
                power = 0.0;
            }

            if (g1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) power = 1.0;
            if (g1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) power = -1.0;
            if (g1.wasJustPressed(GamepadKeys.Button.A)) power = 0.0;

            DcMotorEx m = motors[index];
            m.setPower(power);

            hardware.telemetry.addData("Motor", index);
            hardware.telemetry.addData("Power", power);
            hardware.telemetry.addData("Current", motors[index].getCurrent(CurrentUnit.AMPS));
            hardware.telemetry.addData("Encoder", m.getCurrentPosition());
            hardware.telemetry.addData("Velocity", "%.1f", m.getVelocity());
            hardware.telemetry.update();
        }
    }
}
