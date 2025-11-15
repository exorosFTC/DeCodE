package org.firstinspires.ftc.teamcode.CommandBase.Util;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.util.ElapsedTime;

public class SlewRateLimiter {
    private double m_rateLimit;
    private final ElapsedTime m_timer;
    private double m_prevVal;
    private double m_prevTime;


    public SlewRateLimiter(double rateLimit) {
        m_rateLimit = rateLimit;
        m_prevVal = 0;
        m_prevTime = 0;
        m_timer = new ElapsedTime();
    }

    public double calculate(double input) {
        double currentTime = m_timer.seconds();
        double elapsedTime = currentTime - m_prevTime;
        m_prevVal +=
                MathUtils.clamp(
                        input - m_prevVal,
                        -m_rateLimit * elapsedTime,
                        m_rateLimit * elapsedTime);
        m_prevTime = currentTime;
        return m_prevVal;
    }

    public void setRate(double rate) {
        m_rateLimit = rate;
    }
}