package org.firstinspires.ftc.teamcode.Hardware.Robot.Swerve;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.K_STATIC;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.swerveD;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.swerveP;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.Robot.SystemBase;
import org.firstinspires.ftc.teamcode.Hardware.Util.SensorsEx.AbsoluteAnalogEncoder;


public class SwerveModule extends SystemBase {
    public final DcMotorEx motor;
    public final CRServo servo;
    public final AbsoluteAnalogEncoder encoder;

    private final PIDFController controller = new PIDFController(swerveP, 0, swerveD, 0);
    private boolean wheelFlipped = false;

    public SwerveModuleState currentState = new SwerveModuleState(0, 0);
    public SwerveModuleState targetState = new SwerveModuleState(0, 0);

    public double servoPower;


    public SwerveModule(DcMotorEx motor, CRServo servo, AbsoluteAnalogEncoder encoder) {
        this.motor = motor;
        this.servo = servo;
        this.encoder = encoder;
    }



    public void setDirection(DcMotorSimple.Direction direction) { motor.setDirection(direction); }

    public void setPID(double p, double i, double d) { controller.setPIDF(p, i, d, 0); }

    public void setTargetState(SwerveModuleState state) { targetState = state; }



    public void update() {
        double target = getTargetRotation(), current = getModuleRotation();

        double error = normalizeRadians(target - current);
        if (Math.abs(error) > Math.PI / 2) {
            target = normalizeRadians(target - Math.PI);
            wheelFlipped = true;
        } else {
            wheelFlipped = false;
        }

        error = normalizeRadians(target - current);

        servoPower = Range.clip(controller.calculate(0, error), -1, 1);
        if (Double.isNaN(servoPower)) servoPower = 0;

        servoPower = servoPower + (Math.abs(error) > 0.02 ? K_STATIC : 0) * Math.signum(servoPower);
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
        if (Math.abs(targetState.getModuleVelocity()) < 0.02)
            motor.setMotorDisable();
        else motor.setPower(wheelFlipped ? -targetState.getModuleVelocity() : targetState.getModuleVelocity());
    }
}
