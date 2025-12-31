package org.firstinspires.ftc.teamcode.OpModes.Test.TeleOp.Tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.CommandBase.Constants.Enums;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Hardware;
import org.firstinspires.ftc.teamcode.CommandBase.Util.SensorsEx.AbsoluteAnalogEncoder;

@TeleOp(group = "tune")
public class SwerveOffsetTuner extends LinearOpMode {
    private Hardware hardware;
    private AbsoluteAnalogEncoder LF, LB, RF, RB;

    @Override
    public void runOpMode() throws InterruptedException {
        hardware = Hardware.getInstance(this);

        LF = new AbsoluteAnalogEncoder(hardware.LeftFront_encoder);
        LB = new AbsoluteAnalogEncoder(hardware.LeftBack_encoder);
        RF = new AbsoluteAnalogEncoder(hardware.RightFront_encoder);
        RB = new AbsoluteAnalogEncoder(hardware.RightBack_encoder);

        waitForStart();
        hardware.LeftFront_servo.setPower(0);
        hardware.LeftBack_servo.setPower(0);
        hardware.RightFront_servo.setPower(0);
        hardware.RightBack_servo.setPower(0);

        while (opModeIsActive()) {
            hardware.telemetry.addData("RF, module 1:", RF.getCurrentPosition(AngleUnit.RADIANS));
            hardware.telemetry.addData("LF, module 2:", LF.getCurrentPosition(AngleUnit.RADIANS));
            hardware.telemetry.addData("LB, module 3:", LB.getCurrentPosition(AngleUnit.RADIANS));
            hardware.telemetry.addData("RB, module 4:", RB.getCurrentPosition(AngleUnit.RADIANS));
            hardware.telemetry.update();

            hardware.bulk.clearCache(Enums.Hubs.ALL);
        }
    }
}
