package org.firstinspires.ftc.teamcode.OpModes.Test.TeleOp.Tuning.Configuration;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hardware.Constants.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Hardware;

import java.util.Iterator;
import java.util.Map;

@TeleOp(name = "ConfigureMotor", group = "tuning")
public class MotorTest extends LinearOpMode {
    private DcMotorEx motor;
    private Hardware hardware;

    Iterator<Map.Entry<String, DcMotorEx>> iterator;
    Map.Entry<String, DcMotorEx> current;

    private GamepadEx g2;

    @Override
    public void runOpMode() throws InterruptedException {
        hardware = Hardware.getInstance(this);
        g2 = new GamepadEx(gamepad2);

        iterator = hardware.motors.entrySet().iterator();

        current = iterator.next();
        motor = current.getValue();


        waitForStart();



        while (opModeIsActive()) {
            if (g2.wasJustPressed(GamepadKeys.Button.B)) {

                if (!iterator.hasNext())
                    iterator = hardware.motors.entrySet().iterator();

                current = iterator.next();
                motor = current.getValue();
            }



            if (g2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }





            hardware.telemetry.addData("Motor: ", current.getKey());
            hardware.telemetry.addData("Position: ", motor.getCurrentPosition());
            hardware.telemetry.addData("AMPS", motor.getCurrent(CurrentUnit.AMPS));



            motor.setPower(-g2.getLeftY());

            g2.readButtons();
            hardware.bulk.clearCache(Enums.Hubs.ALL);
            hardware.telemetry.update();
        }
    }
}
