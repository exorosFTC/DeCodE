package org.firstinspires.ftc.teamcode.CommandBase.Robot.Swerve;

import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoSwerveModuleD;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoSwerveModuleP;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.TeleOpSwerveModuleD;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.TeleOpSwerveModuleP;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.SystemConstants.opModeType;
import static org.firstinspires.ftc.teamcode.CustomPathing.Math.MathFormulas.normalizeRadians;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.CommandBase.Constants.SystemConstants;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.SystemBase;
import org.firstinspires.ftc.teamcode.CommandBase.Util.SensorsEx.AbsoluteAnalogEncoder;


public class SwerveModule extends SystemBase {
    public final DcMotorEx motor;
    public final CRServo servo;
    public final AbsoluteAnalogEncoder encoder;

    private double kS_angle, kS_threshold;

    private final PIDFController controller;
    private boolean wheelFlipped = false;

    public SwerveModuleState currentState = new SwerveModuleState(0, 0);
    public SwerveModuleState targetState = new SwerveModuleState(0, 0);

    public double servoPower;
    public double error;


    public SwerveModule(DcMotorEx motor, CRServo servo, AbsoluteAnalogEncoder encoder,
                        double kS_angle, double kS_threshold) {
        this.motor = motor;
        this.servo = servo;
        this.encoder = encoder;

        this.kS_angle = kS_angle;
        this.kS_threshold = kS_threshold;

        controller = new PIDFController(
                opModeType == SystemConstants.OpMode.AUTONOMOUS ? AutoSwerveModuleP : TeleOpSwerveModuleP,
                0,
                opModeType == SystemConstants.OpMode.AUTONOMOUS ? AutoSwerveModuleD : TeleOpSwerveModuleD,
                0);
    }



    public void setDirection(DcMotorSimple.Direction direction) { motor.setDirection(direction); }

    public void setPID(double p, double i, double d) { controller.setPIDF(p, i, d, 0); }

    public void setTargetState(SwerveModuleState state) { targetState = state; }

    public void setAngleKS(double kS) { this.kS_angle = kS; }



    public void update() {
        double target = getTargetRotation(), current = getModuleRotation();

        error = normalizeRadians(target - current);
        if (Math.abs(error) > Math.PI / 2) {
            target = normalizeRadians(target - Math.PI);
            wheelFlipped = true;
        } else { wheelFlipped = false; }

        error = normalizeRadians(target - current);

        servoPower = Range.clip(controller.calculate(0, error), -1, 1);
        if (Double.isNaN(servoPower)) servoPower = 0;

        servoPower = servoPower + (Math.abs(error) > kS_threshold ? kS_angle : 0) * Math.signum(servoPower);
    }

    public double getTargetRotation() {
        return normalizeRadians(targetState.getModuleAngle() - Math.PI);
    }

    public double getModuleRotation() {
        return normalizeRadians(currentState.getModuleAngle() - Math.PI);
    }

    @Override
    public void read() {
        currentState.setModuleAngle(encoder.getCurrentPosition(AngleUnit.RADIANS));
        currentState.setModuleVelocity(motor.getPower());
    }

    @Override
    public void write() {
        servo.setPower(servoPower);
        motor.setPower(Math.abs(targetState.getModuleVelocity()) > 0.08 ? ((wheelFlipped ? -1 : 1) * targetState.getModuleVelocity()) : 0);
    }
}
