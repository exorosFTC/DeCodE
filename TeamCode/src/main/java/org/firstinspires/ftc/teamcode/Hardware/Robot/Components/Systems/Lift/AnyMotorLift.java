package org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Lift;


import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.manualLiftCoefficient;


import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Util.MotionHardware.Init;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;

public class AnyMotorLift {
    public Hardware hardware;
    private LinearOpMode opMode;

    public final Map<String, Integer> values = new HashMap<>();
    public final List<String> motors;
    public final String encoder;

    private int MIN = 0, MAX = 0;
    private int position = 0;
    private double amps = 1.5;
    private int diff;

    private double multiplier = 1;

    private double f = 0.3;

    private double alpha = 1;
    private double filteredPower = 0;

    private final PIDController controller;
    private double PIDthreshold = 0;

    private boolean ENABLED = true;
    private boolean wasReset = false;



    public AnyMotorLift(LinearOpMode opMode, List<String> motors, String encoder) {
        this.hardware = Hardware.getInstance(opMode);
        this.opMode = opMode;

        controller = new PIDController(0, 0, 0);

        this.motors = motors;
        this.encoder = encoder;

        resetEncoders();
    }


    public AnyMotorLift add(String key, int value) {
        values.put(key, value);
        return this;
    }

    public AnyMotorLift setMultiplierNegative(boolean flag) {
        if (flag) multiplier = -1;
        else multiplier = 1;

        return this;
    }

    public AnyMotorLift addLimit(int MAX) {
        this.MAX = MAX;
        diff = this.MAX - this.MIN;

        return this;
    }

    public AnyMotorLift addLimit(int MIN, int MAX) {
        this.MIN = MIN;
        this.MAX = MAX;
        diff = this.MAX - this.MIN;

        return this;
    }

    public AnyMotorLift addCurrentAlert(double amps) {
        this.amps = amps;

        for (String motor : motors)
            hardware.motors.get(motor).setCurrentAlert(amps, CurrentUnit.AMPS);

        return this;
    }

    public AnyMotorLift addPIDThreshold(double threshold) {
        this.PIDthreshold = threshold;
        return this;
    }

    public AnyMotorLift reverse(String motor) {
        hardware.motors.get(motor).setDirection(DcMotorSimple.Direction.REVERSE);
        return this;
    }

    public AnyMotorLift setControllerPID(double p, double i, double d, double f) {
        controller.setPID(p, i, d);
        this.f = f;

        return this;
    }

    public AnyMotorLift setAlpha(double alpha) {
        this.alpha = alpha;
        return this;
    }


    public void resetEncoders() {
        hardware.motors.put(encoder, Init.initializeMotor(hardware.motors.get(encoder), DcMotor.RunMode.RUN_WITHOUT_ENCODER));
        }



    public void setPosition(String key) { position = values.get(key).intValue();}

    public void setPosition(int pos) { position = Math.min(MAX, Math.max(MIN, pos)); }


    public void extend(double power) {
        extend(power, false);
    }

    // input [-1, 1] ---- joystick input
    public void extend(double power, boolean noLimit) {
        int newPosition = this.position + (int) (this.diff * manualLiftCoefficient * power);

        if (!noLimit)
            position = Math.min(MAX, Math.max(MIN, newPosition));
        else position = newPosition;

        ENABLED = true;
    }

    public int getPosition() {
        return hardware.motors.get(encoder).getCurrentPosition();
    }


    public void disable() {
        for (String motor : motors)  hardware.motors.get(motor).setMotorDisable();
        ENABLED = false;
    }

    public void autoReset() {
        ElapsedTime timer = new ElapsedTime();
        while (opMode.opModeIsActive() && !constrained() && timer.time(TimeUnit.MILLISECONDS) < 1200) {
            hardware.bulk.clearCache(Enums.Hubs.ALL);
            extend(-1, true);
            update();
        }

        extend(0);
        resetEncoders();
        wasReset = true;
    }

    public boolean constrained() {
        for (String motor : motors)
            if (hardware.motors.get(motor).isOverCurrent()) return true;

        return false;
    }

    public void update() {
        if (!SystemConstants.updateOuttake || reached(PIDthreshold)) {
            for (String motor : motors) hardware.motors.get(motor).setMotorDisable();
            return;
        }

        if (!reached(PIDthreshold)) {
            int currentPosition = hardware.motors.get(encoder).getCurrentPosition();

            double power = controller.calculate(currentPosition, position);
            power = lowPassFilter(Math.max(Math.min(power + Math.signum(power) * f, 1), -1) * multiplier);

            for (String motor : motors) hardware.motors.get(motor).setPower(power);
        } else for (String motor : motors) hardware.motors.get(motor).setPower(0);


    }

    public boolean reached() { return reached(2); }

    public boolean reached(double threshold) {
        return Math.abs(hardware.motors.get(encoder).getCurrentPosition() - position) <= threshold;
    }

    public double lowPassFilter(double input) {
        filteredPower = alpha * input + (1 - alpha) * filteredPower;
        return filteredPower;
    }
}
