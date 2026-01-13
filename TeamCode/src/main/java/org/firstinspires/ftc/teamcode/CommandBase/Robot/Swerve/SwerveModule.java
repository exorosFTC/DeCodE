package org.firstinspires.ftc.teamcode.CommandBase.Robot.Swerve;

import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.swerveModuleD;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.swerveModuleP;
import static org.firstinspires.ftc.teamcode.CustomPathing.Math.MathFormulas.normalizeRadians;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Hardware;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.SystemBase;
import org.firstinspires.ftc.teamcode.CommandBase.Util.SensorsEx.AbsoluteAnalogEncoder;


public class SwerveModule extends SystemBase {
    public final DcMotorEx motor;
    public final CRServo servo;
    public final AbsoluteAnalogEncoder encoder;

    private double kS_wheel;
    private double kS_angle, kS_threshold;

    private final PIDFController controller = new PIDFController(swerveModuleP, 0, swerveModuleD, 0);
    private boolean wheelFlipped = false;

    public SwerveModuleState currentState = new SwerveModuleState(0, 0);
    public SwerveModuleState targetState = new SwerveModuleState(0, 0);

    public double servoPower;


    public SwerveModule(DcMotorEx motor, CRServo servo, AbsoluteAnalogEncoder encoder,
                        double kS_wheel, double kS_angle, double kS_threshold) {
        this.motor = motor;
        this.servo = servo;
        this.encoder = encoder;

        this.kS_wheel = kS_wheel;
        this.kS_angle = kS_angle;
        this.kS_threshold = kS_threshold;
    }



    public void setDirection(DcMotorSimple.Direction direction) { motor.setDirection(direction); }

    public void setPID(double p, double i, double d) { controller.setPIDF(p, i, d, 0); }

    public void setTargetState(SwerveModuleState state) { targetState = state; }



    public void update(Hardware hardware) {
        double target = getTargetRotation(), current = getModuleRotation();

        double error = normalizeRadians(target - current);
        if (Math.abs(error) > Math.PI / 2) {
            target = normalizeRadians(target - Math.PI);
            wheelFlipped = true;
        } else { wheelFlipped = false; }

        error = normalizeRadians(target - current);
        hardware.telemetry.addData("error", error);

        servoPower = Range.clip(controller.calculate(0, error), -1, 1);
        if (Double.isNaN(servoPower)) servoPower = 0;

        servoPower = servoPower + (Math.abs(error) > kS_threshold ? kS_angle : 0) * Math.signum(servoPower);
        //targetState.setModuleVelocity(
              //Math.signum(targetState.getModuleVelocity()) * kS_wheel +             // static friction correction
              //Math.max(Math.min(targetState.getModuleVelocity(), 0.88), -0.88)
              //* Math.cos(Math.abs(error))                                         // cosine correction
        //);
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
        if (Math.abs(targetState.getModuleVelocity()) > 0.02)
            motor.setPower(wheelFlipped ? -targetState.getModuleVelocity() : targetState.getModuleVelocity());
        else motor.setPower(0);
    }
}
