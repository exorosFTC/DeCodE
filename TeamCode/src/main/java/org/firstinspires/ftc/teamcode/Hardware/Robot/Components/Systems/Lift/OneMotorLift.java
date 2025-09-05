package org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Lift;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.manualLiftCoefficient;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.TimeUnit;

public class OneMotorLift {
    private static Hardware hardware;
    private static LinearOpMode opMode;

    public final Map<String, Integer> values = new HashMap<>();
    private DcMotor.RunMode runMode = DcMotor.RunMode.RUN_TO_POSITION;

    private int MIN = 0, MAX = 0;
    private double amps = 1.5;
    private int diff;

    private final String motor;
    private int position;




    public OneMotorLift(LinearOpMode opMode, String motor) {
        this.hardware = Hardware.getInstance(opMode);
        this.opMode = opMode;

        this.motor = motor;
    }



    public OneMotorLift add(String key, int value) {
        values.put(key, Integer.valueOf(value));
        return this;
    }

    public OneMotorLift addLimit(int MAX) {
        this.MAX = MAX;
        diff = this.MAX - this.MIN;

        return this;
    }

    public OneMotorLift addLimit(int MIN, int MAX) {
        this.MIN = MIN;
        this.MAX = MAX;
        diff = this.MAX - this.MIN;

        return this;
    }

    public OneMotorLift addCurrentAlert(double amps) {
        this.amps = amps;

        hardware.motors.get(motor).setCurrentAlert(amps, CurrentUnit.AMPS);

        return this;
    }

    public OneMotorLift reverse() {
        hardware.motors.get(motor).setDirection(DcMotorSimple.Direction.REVERSE);
        return this;
    }



    public void setPosition(String key) {
        position = Math.min(MAX, Math.max(MIN, values.get(key).intValue()));

        hardware.motors.get(motor).setTargetPosition(position);
        hardware.motors.get(motor).setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.motors.get(motor).setPower(1);
    }

    public void setPosition(int position) {
        this.position = position;

        hardware.motors.get(motor).setTargetPosition(position);
        hardware.motors.get(motor).setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.motors.get(motor).setPower(1);
    }

    public void extend(double power) {
        extend(power, false);
    }

    // input [-1, 1] ---- joystick input
    public void extend(double power, boolean noLimit) {
        if (runMode == DcMotor.RunMode.RUN_WITHOUT_ENCODER)
            { hardware.motors.get(motor).setPower(power); return; }

        int pastPosition = hardware.motors.get(motor).getCurrentPosition();
        position = pastPosition + (int) (this.diff * manualLiftCoefficient * power);

        if (!noLimit)
            position = Math.min(MAX, position);

        hardware.motors.get(motor).setTargetPosition(position);
        hardware.motors.get(motor).setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.motors.get(motor).setPower(1);
    }

    public int getPosition() {
        return hardware.motors.get(motor).getCurrentPosition();
    }

    public void disable() { hardware.motors.get(motor).setMotorDisable(); }

    public void autoReset() {
        runWithoutEncoder();
        extend(-1);

        ElapsedTime timer = new ElapsedTime();

        while (opMode.opModeIsActive() && !constrained() && timer.time(TimeUnit.MILLISECONDS) < 1400) {
            hardware.bulk.clearCache(Enums.Hubs.ALL);

            hardware.telemetry.addData("pose: ", getPosition());
            hardware.telemetry.addData("current: ", getCurrent());
            hardware.telemetry.update();
        }

        extend(0);
        resetEncoders();
        runToPosition();
    }




    public void resetEncoders() {
        hardware.motors.get(motor).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runToPosition() {
        hardware.motors.get(motor).setTargetPosition(getPosition());
        hardware.motors.get(motor).setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.motors.get(motor).setPower(1);

        runMode = DcMotor.RunMode.RUN_TO_POSITION;
    }

    public void runWithoutEncoder() {
        hardware.motors.get(motor).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    }




    public boolean constrained() {
        return hardware.motors.get(motor).isOverCurrent();
    }

    public boolean reached() { return reached(5); }

    public boolean reached(double threshold) {
        return Math.abs(hardware.motors.get(motor).getCurrentPosition() - position) <= threshold;
    }




    public double getCurrent() { return hardware.motors.get(motor).getCurrent(CurrentUnit.AMPS); }

}
