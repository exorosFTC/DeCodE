package org.firstinspires.ftc.teamcode.Hardware.Robot.Scoring;

class ShooterState {
    public double targetRpsCmd = 0.0;
    public double targetRpsSlew = 0.0;

    public double lastRps1 = 0.0;
    public double lastRps2 = 0.0;

    public double errI1 = 0.0;
    public double errI2 = 0.0;

    public double lastErr1 = 0.0;
    public double lastErr2 = 0.0;

    public long lastTimeNanos = 0;

    public ShooterState() {}

    public void reset() {
        targetRpsCmd = 0.0;
        targetRpsSlew = 0.0;
        lastRps1 = 0.0;
        lastRps2 = 0.0;
        errI1 = 0.0;
        errI2 = 0.0;
        lastErr1 = 0.0;
        lastErr2 = 0.0;
        lastTimeNanos = 0;
    }
}
