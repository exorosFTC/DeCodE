package org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.Subsystems;

import static androidx.core.math.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.POSE;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.goalPosition;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.SystemConstants.samples;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Hardware;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.ShotSample;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.SystemBase;

public class Shooter extends SystemBase {
    private final Hardware hardware;
    private final PIDFController controller;

    public static double kP = 0.009;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0.00155;

    private static final double VEL_ALPHA = 0.3;
    public static final double COAST_POWER = 0;
    public static final double MAX_RPS = 600;
    public static double ANGLE_ADJUST = 0;

    public boolean enabled = true;




    public double distance;
    public double rawVelocity = 0;
    public double wheelVelocity = 0;

    public double targetPower = 0;
    public double targetAngle = 0.97;

    public double POWER = 0;
    public double TARGET = 0;

    private final double threshold = 6;


    public Shooter(LinearOpMode opMode) {
        this.hardware = Hardware.getInstance(opMode);
        controller = new PIDFController(kP, kI, kD, kF);
    }

    public boolean ready() { return ready(threshold); }

    public boolean ready(double threshold) {
        return Math.abs(this.TARGET - wheelVelocity) < threshold;
    }


    public void update() {
        distance = POSE.distanceTo(goalPosition);

        if (!enabled) return;
        if (on) {
            ShotSample shot = lookupShot(distance);

            if (shot != null) {
                targetPower = shot.power;
                targetAngle = shot.angle;
            }
        }

        this.TARGET = targetPower * MAX_RPS;
        this.POWER = controller.calculate(wheelVelocity, TARGET);

        if (!on) return;
        targetAngle = clamp(targetAngle - (this.TARGET - wheelVelocity - threshold) * ANGLE_ADJUST, 0.37, 0.97);  // adjust angle by velocity*/
    }

    private ShotSample lookupShot(double d) {
        if (samples == null || samples.isEmpty()) return null;

        ShotSample before = null;
        ShotSample after  = null;

        // find closest sample <= d  and closest sample >= d
        for (ShotSample s : samples) {
            if (s.distance <= d) {
                if (before == null || s.distance > before.distance) {
                    before = s;
                }
            }
            if (s.distance >= d) {
                if (after == null || s.distance < after.distance) {
                    after = s;
                }
            }
        }

        if (before == null) return after;
        if (after == null) return before;
        if (before.distance == after.distance) {
            return before;
        }

        double t = (d - before.distance) / (after.distance - before.distance);

        double power = lerp(before.power, after.power, t);
        double angle = lerp(before.angle, after.angle, t);

        return new ShotSample(d, power, angle);
    }

    private double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    public void setPIDF(double p, double i, double d, double f) {
        controller.setPIDF(p, i, d, f);
    }



    @Override
    public void read() {
        rawVelocity = -hardware.Shooter2.getVelocity(AngleUnit.DEGREES);
        wheelVelocity += VEL_ALPHA * (rawVelocity - wheelVelocity);
    }

    @Override
    public void write() {
        if (!enabled) return;

        hardware.Shooter1.setPower(this.POWER);
        hardware.Shooter2.setPower(-this.POWER);

        if (!on) return;
        hardware.ShooterHoodServo.setPosition(targetAngle);
    }

    @Override
    public void on() {
        enabled = true;
        super.on();
    }

    @Override
    public void off() {
        super.off();

        targetPower = COAST_POWER;
        hardware.ShooterHoodServo.getController().pwmDisable();
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
        if (enabled) return;

        hardware.Shooter1.setMotorDisable();
        hardware.Shooter2.setMotorDisable();
        hardware.ShooterHoodServo.getController().pwmDisable();
    }
}


