package org.firstinspires.ftc.teamcode.Hardware.Robot.Drivetrain.Swerve;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.K_STATIC;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.swerveD;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.swerveP;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.Util.SensorsEx.AbsoluteAnalogEncoder;


public class SwerveModule {
    private final DcMotorEx motor;
    private final CRServo servo;
    public final AbsoluteAnalogEncoder encoder;
    private final PIDFController controller = new PIDFController(swerveP, 0, swerveD, 0);

    private boolean wheelFlipped = false;

    public SwerveModuleState currentState = new SwerveModuleState(0, 0);
    public SwerveModuleState targetState = new SwerveModuleState(0, 0);




    public SwerveModule(DcMotorEx motor, CRServo servo, AbsoluteAnalogEncoder encoder) {
        this.motor = motor;
        this.servo = servo;
        this.encoder = encoder;
    }



    public void setDirection(DcMotorSimple.Direction direction) { motor.setDirection(direction); }

    public void setPID(double p, double i, double d) { controller.setPIDF(p, i, d, 0); }

    public void setTargetState(SwerveModuleState state) { targetState = state; }



    public void update() {
        currentState.setModuleAngle(encoder.getCurrentPosition(AngleUnit.RADIANS));
        currentState.setModuleVelocity(motor.getPower());

        double target = getTargetRotation(), current = getModuleRotation();

        double error = normalizeRadians(target - current);
        if (Math.abs(error) > Math.PI / 2) {
            target = normalizeRadians(target - Math.PI);
            wheelFlipped = true;
        } else {
            wheelFlipped = false;
        }

        error = normalizeRadians(target - current);

        double power = Range.clip(controller.calculate(0, error), -1, 1);
        if (Double.isNaN(power)) power = 0;

        servo.setPower(power + (Math.abs(error) > 0.02 ? K_STATIC : 0) * Math.signum(power));
        motor.setPower(wheelFlipped ? -targetState.getModuleVelocity() : targetState.getModuleVelocity());
    }

    public double getTargetRotation() {
        return normalizeRadians(targetState.getModuleAngle() - Math.PI);
    }

    public double getModuleRotation() {
        return normalizeRadians(currentState.getModuleAngle() - Math.PI);
    }
}
