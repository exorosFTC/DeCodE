package org.firstinspires.ftc.teamcode.OpModes.Test.TeleOp.Tuning;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.CommandBase.Robot.Hardware;

@TeleOp(group = "test")
public class ServoTest extends LinearOpMode {

    private GamepadEx g1;
    private Hardware hardware;

    private Servo[] servos;
    private int servoIndex = 0;

    private double position = 0;
    private static final double STEP = 0.01;

    @Override
    public void runOpMode() {
        hardware = Hardware.getInstance(this);
        g1 = new GamepadEx(gamepad1);

        servos = new Servo[]{
                hardware.ShooterHoodServo,
                hardware.LiftRetainerServo
        };

        waitForStart();

        while (opModeIsActive()) {
            g1.readButtons();

            if (g1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) { position += STEP; }
            if (g1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) { position -= STEP; }

            position = Math.max(0.0, Math.min(1.0, position));

            if (g1.wasJustPressed(GamepadKeys.Button.B)) { servoIndex = (servoIndex + 1) % servos.length; }

            servos[servoIndex].setPosition(position);

            hardware.telemetry.addData("Active Servo", servoIndex);
            hardware.telemetry.addData("Position", "%.3f", position);
            hardware.telemetry.addData("Servo Name", servos[servoIndex].getDeviceName());
            hardware.telemetry.update();
        }
    }
}