package org.firstinspires.ftc.teamcode.CommandBase.Util;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.util.ElapsedTime;

public class SlewRateLimiter {
    private final ElapsedTime timer = new ElapsedTime();

    private double prev = 0.0;
    private double prevTime = 0.0;

    // units per second (for joystick, this is "per second" in [-1..1] space)
    private double accelRate; // when magnitude should increase
    private double decelRate; // when magnitude should decrease (usually higher = stops faster)

    /**
     * @param seconds0to1Accel  seconds to go from 0 -> 1 when speeding up
     * @param seconds0to1Decel  seconds to go from 1 -> 0 when slowing down
     */
    public SlewRateLimiter(double seconds0to1Accel, double seconds0to1Decel) {
        setSeconds(seconds0to1Accel, seconds0to1Decel);
        prevTime = timer.seconds();
    }

    public void setSeconds(double seconds0to1Accel, double seconds0to1Decel) {
        accelRate = (seconds0to1Accel <= 0) ? Double.POSITIVE_INFINITY : 1.0 / seconds0to1Accel;
        decelRate = (seconds0to1Decel <= 0) ? Double.POSITIVE_INFINITY : 1.0 / seconds0to1Decel;
    }

    public void setRates(double accelPerSec, double decelPerSec) {
        accelRate = accelPerSec;
        decelRate = decelPerSec;
    }

    public void reset(double value) {
        prev = value;
        prevTime = timer.seconds();
    }

    public double get() {
        return prev;
    }

    public double calculate(double target) {
        double now = timer.seconds();
        double dt = now - prevTime;

        if (dt <= 0 || dt > 0.1) dt = 0.02;

        double error = target - prev;
        boolean increasingMagnitude = Math.abs(target) > Math.abs(prev) && Math.signum(target) == Math.signum(prev);

        if (Math.signum(target) != Math.signum(prev) && prev != 0.0) {
            increasingMagnitude = false;
        }

        double rate = increasingMagnitude ? accelRate : decelRate;
        double maxStep = rate * dt;

        double step = MathUtils.clamp(error, -maxStep, +maxStep);
        prev += step;

        prevTime = now;
        return prev;
    }
}