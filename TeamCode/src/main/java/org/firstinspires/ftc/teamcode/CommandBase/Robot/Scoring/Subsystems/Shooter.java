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

    public static double kP = 0.05;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0.002;

    private static final double VEL_ALPHA = 0.3;
    public static final double MAX_RPS = 600;
    public static double ANGLE_ADJUST = -0.002;
    public static double VELOCITY_ADJUST = 0;



    public double distance;

    public double rawVelocity = 0;
    public double correctedVelocity = 0;
    public double targetVelocity = 0;

    public double currentPower = 0;
    public double targetPower = 0;

    public double targetAngle = 0.94;
    private final double threshold = 6;


    public Shooter(LinearOpMode opMode) {
        this.hardware = Hardware.getInstance(opMode);
        controller = new PIDFController(kP, kI, kD, kF);
    }

    public boolean ready() { return ready(threshold); }

    public boolean ready(double threshold) {
        return Math.abs(this.targetVelocity - correctedVelocity) < threshold;
    }


    public void update() { update(-1, -1);}

    public void update(double overridePower, double overrideAngle) {
        distance = POSE.distanceTo(goalPosition);
        if (!on) return;


        ShotSample shot = lookupShot(distance);

        if (overridePower != -1) {
            targetPower = overridePower;
            targetAngle = overrideAngle;
        } else if (shot != null) {
            targetPower = shot.power;
            targetAngle = shot.angle;
        }

        this.targetVelocity = targetPower * MAX_RPS;
        this.currentPower = controller.calculate(correctedVelocity, targetVelocity) + VELOCITY_ADJUST * Math.abs(targetVelocity - correctedVelocity);

        targetAngle = clamp(targetAngle - (this.targetVelocity - correctedVelocity - threshold) * ANGLE_ADJUST, 0.34, 0.94);  // adjust angle by velocity*/
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
        correctedVelocity += VEL_ALPHA * (rawVelocity - correctedVelocity);
    }

    @Override
    public void write() {
        if (!on) return;
        hardware.Shooter1.setPower(this.currentPower);
        hardware.Shooter2.setPower(-this.currentPower);

        hardware.ShooterHoodServo.setPosition(targetAngle);
    }

    @Override
    public void off() {
        super.off();
        targetPower = 0;
        targetVelocity = 0;
        currentPower = 0;
        targetAngle = 0.94;

        hardware.Shooter1.setPower(0);
        hardware.Shooter2.setPower(0);
        hardware.ShooterHoodServo.setPosition(0.94);

    }
}


