package org.firstinspires.ftc.teamcode.OpModes.Test.TeleOp.Tuning;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.CommandBase.Robot.Hardware;

@TeleOp(group = "test")
public class CRServoTest extends LinearOpMode {

    private GamepadEx g1;
    private Hardware hardware;

    private CRServo[] crServos;
    private int servoIndex = 0;

    private double power = 0.0;

    @Override
    public void runOpMode() {
        hardware = Hardware.getInstance(this);
        g1 = new GamepadEx(gamepad1);

        crServos = new CRServo[]{
                hardware.LeftFront_lift,
                hardware.LeftBack_lift,
                hardware.RightFront_lift,
                hardware.RightBack_lift,
                hardware.LeftFront_servo,
                hardware.LeftBack_servo,
                hardware.RightFront_servo,
                hardware.RightBack_servo
        };

        waitForStart();

        while (opModeIsActive()) {
            g1.readButtons();

            // FULL POWER CONTROL
            if (g1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                power = 1.0;
            }

            if (g1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                power = -1.0;
            }

            if (g1.wasJustPressed(GamepadKeys.Button.A)) {
                power = 0.0;
            }

            // Switch servo
            if (g1.wasJustPressed(GamepadKeys.Button.B)) {
                crServos[servoIndex].setPower(0);
                servoIndex = (servoIndex + 1) % crServos.length;
                power = 0.0; // safety stop on switch
            }

            // Apply power
            power = g1.getLeftY();
            crServos[servoIndex].setPower(power);

            // Telemetry
            hardware.telemetry.addData("Active CRServo", servoIndex);
            hardware.telemetry.addData("Power", power);
            hardware.telemetry.addData("Device", crServos[servoIndex].getDeviceName());
            hardware.telemetry.addLine("DPAD_UP = +1 | DPAD_DOWN = -1 | A = stop | B = next servo");
            hardware.telemetry.update();
        }
    }
}
