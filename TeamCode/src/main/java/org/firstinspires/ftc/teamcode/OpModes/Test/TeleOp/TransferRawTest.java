package org.firstinspires.ftc.teamcode.OpModes.Test.TeleOp;

import static org.firstinspires.ftc.teamcode.Hardware.Constants.HardwareNames.IndexerMotor;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.HardwareNames.IntakeMotor;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.HardwareNames.ShooterMotor1;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.HardwareNames.ShooterMotor2;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Robot.Hardware;

@TeleOp(name = "Transfer Raw Test", group = "Test")
public class TransferRawTest extends LinearOpMode {
    Hardware hardware;
    GamepadEx g2;

    RevBlinkinLedDriver led;

    boolean shooting = false;
    boolean intaking = false;

    @Override
    public void runOpMode() throws InterruptedException {
        hardware = Hardware.getInstance(this);
        g2 = new GamepadEx(gamepad2);

        waitForStart();

        while (opModeIsActive()) {
            hardware.motors.get(IndexerMotor).setPower(g2.getLeftY());

            if (g2.wasJustPressed(GamepadKeys.Button.A)) {
                shooting = !shooting;

                if (shooting) {
                    hardware.motors.get(ShooterMotor1).setPower(1);
                    hardware.motors.get(ShooterMotor2).setPower(-1);
                } else {
                    hardware.motors.get(ShooterMotor1).setPower(0);
                    hardware.motors.get(ShooterMotor2).setPower(0);
                }
            }

            if (g2.wasJustPressed(GamepadKeys.Button.B)) {
                intaking = !intaking;

                if (intaking) {
                    hardware.motors.get(IntakeMotor).setPower(1);
                } else {
                    hardware.motors.get(IntakeMotor).setPower(0);
                }
            }

            g2.readButtons();
        }
    }
}
