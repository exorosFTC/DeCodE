package org.firstinspires.ftc.teamcode.OpModes.Test.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Hardware;

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
            hardware.IndexerMotor.setPower(g2.getLeftY());

            if (g2.wasJustPressed(GamepadKeys.Button.A)) {
                shooting = !shooting;

                if (shooting) {
                    hardware.Shooter1.setPower(0.5);
                    hardware.Shooter2.setPower(-0.5);
                } else {
                    hardware.Shooter1.setPower(0);
                    hardware.Shooter2.setPower(0);
                }
            }

            if (g2.wasJustPressed(GamepadKeys.Button.B)) {
                intaking = !intaking;

                if (intaking) {
                    hardware.IntakeMotor.setPower(-1);
                } else {
                    hardware.IntakeMotor.setPower(0);
                }
            }

            hardware.ShooterHoodServo.setPosition(
                    (-g2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) + g2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)) * 0.1 +
                            hardware.ShooterHoodServo.getPosition()
            );

            hardware.telemetry.addData("Shooter1 AMPS", hardware.Shooter1.getCurrent(CurrentUnit.AMPS));
            hardware.telemetry.addData("Shooter2 AMPS", hardware.Shooter2.getCurrent(CurrentUnit.AMPS));
            hardware.telemetry.addData("indexer pos", hardware.IndexerMotor.getCurrentPosition());
            hardware.telemetry.update();

            g2.readButtons();
        }
    }
}
