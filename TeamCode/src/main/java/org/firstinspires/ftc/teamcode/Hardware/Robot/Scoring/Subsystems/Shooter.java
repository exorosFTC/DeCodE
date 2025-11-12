package org.firstinspires.ftc.teamcode.Hardware.Robot.Scoring.Subsystems;

import static androidx.core.math.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.POSE;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.goalPosition;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot.SystemBase;

public class Shooter extends SystemBase {
    private final Hardware hardware;
    private final PIDController controller;

    public double distance;
    public double wheelVelocity = 0;

    public static final double MAX_RPS = 600;
    public static double c2_angle_adjust = 0.006,
                         c_angle_close = 0.0015,
                         c_angle_far = 0,
                         c_power = 0.0026;

    public double targetPower = 0;
    public double targetAngle = 0;


    private double POWER = 0;
    public double TARGET = 0;

    private final double threshold = 9;

    public double kP = 12e-4;
    public double kI = 0;
    public double kD = 9e-5;


    public Shooter(LinearOpMode opMode) {
        this.hardware = Hardware.getInstance(opMode);
        controller = new PIDController(kP, kI, kD);
    }

    public boolean ready() { return ready(threshold); }

    public boolean ready(double threshold) {
        return Math.abs(this.TARGET - wheelVelocity) < threshold;
    }



    public void update() {
        if (!on) return;

        hardware.telemetry.addData("velocity", wheelVelocity);
        hardware.telemetry.addData("velocity target", TARGET);

        distance = POSE.distanceTo(goalPosition);

        targetPower();
        targetAngle();

        this.POWER = this.POWER + controller.calculate(wheelVelocity, TARGET);
    }


    private void targetPower() {
        if (distance < 300) {
            targetPower = clamp(distance * c_power, 0, 1);
        } else targetPower = 0.95;

        this.TARGET = targetPower * MAX_RPS;
    }

    private void targetAngle() {
        if (distance < 300) {
            targetAngle = clamp(distance * c_angle_close - (this.TARGET - threshold) * c2_angle_adjust, 0.05, 1);
        } else {
            targetAngle = clamp(distance * c_angle_far - (this.TARGET- threshold) * c2_angle_adjust, 0.05, 1);
        }
    }




    @Override
    public void read() {
        if (!on) return;

        wheelVelocity = hardware.Shooter2.getVelocity(AngleUnit.DEGREES);
    }

    @Override
    public void write() {
        if (!on) return;

        hardware.Shooter1.setPower(this.POWER);
        hardware.Shooter2.setPower(this.POWER);
        hardware.ShooterHoodServo.setPosition(targetAngle);
    }

    @Override
    public void off() {
        super.off();

        this.targetPower = 0;
        this.TARGET = 0;
        this.POWER = 0;

        hardware.Shooter1.setMotorDisable();
        hardware.Shooter2.setMotorDisable();

        hardware.ShooterHoodServo.getController().pwmDisable();
    }
}


