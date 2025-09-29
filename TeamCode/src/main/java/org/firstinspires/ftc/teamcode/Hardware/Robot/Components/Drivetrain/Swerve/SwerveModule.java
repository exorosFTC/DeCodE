package org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Drivetrain.Swerve;

import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.swerveD;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.swerveP;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.useMotorFlipping;
import static org.firstinspires.ftc.teamcode.Pathing.Math.MathFormulas.clip;

import static java.lang.Math.signum;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Util.SensorsEx.AbsoluteAnalogEncoderEx;


public class SwerveModule {
    private final DcMotorEx motor;
    private final CRServo servo;
    private final AbsoluteAnalogEncoderEx encoder;

    private final PIDFController controller = new PIDFController(swerveP, 0, swerveD, 0);
    private boolean wheelFlipped = false;


    private final SwerveModuleState currentState = new SwerveModuleState(0, 0);
    private SwerveModuleState targetState = new SwerveModuleState(0, 0);



    public SwerveModule(HardwareMap hardwareMap, String motor_name, String servo_name, String encoder_name) {
        motor = hardwareMap.get(DcMotorEx.class, motor_name);
        servo = hardwareMap.get(CRServo.class, servo_name);
        encoder = new AbsoluteAnalogEncoderEx(hardwareMap.get(AnalogInput.class, encoder_name));

        initialize();
    }

    public SwerveModule(DcMotorEx motor, CRServo servo, AbsoluteAnalogEncoderEx encoder) {
        this.motor = motor;
        this.servo = servo;
        this.encoder = encoder;

        // uncomment if you didn't initialize them properly before
        // initialize();
    }



    public void setDirection(DcMotorSimple.Direction direction) { motor.setDirection(direction); }

    public void setPID(double p, double i, double d) { controller.setPIDF(p, i, d, 0); }

    public void setTargetState(SwerveModuleState state) { targetState = state; }

    public void set(double s, double a) {
        double p = (s > 0.02) ? (clip(s * flipModifier(), -1, 1)) : 0;

        targetState.setModuleAngle(a);
        targetState.setModuleVelocity(p);
        currentState.setModuleVelocity(p);

        motor.setPower(p);
    }



    public void update() {
        currentState.setModuleAngle(encoder.getCurrentPosition(AngleUnit.RADIANS));
        currentState.setModuleVelocity(motor.getPower());

        // both are in the [-π, +π] range
        double error = targetState.getModuleAngle() - currentState.getModuleAngle();

        if (Math.abs(error) > Math.PI / 2.0 && useMotorFlipping) {
            wheelFlipped = !wheelFlipped;
            error = error > 0 ? error - Math.PI : error + Math.PI;
        }

        double servo_pow = controller.calculate(error);
        double motor_pow = (targetState.getModuleVelocity() > 0.02) ? (clip(targetState.getModuleVelocity() * flipModifier(), -1, 1)) : 0;

        servo.setPower(servo_pow);
        motor.setPower(motor_pow);
    }



    public void initialize() {
        MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1);
        motor.setMotorType(motorConfigurationType);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ((ServoImplEx) servo).setPwmRange(new PwmControl.PwmRange(500, 2500, 5000));
    }

    public int flipModifier() { return wheelFlipped ? -1 : 1; }

    public boolean isFlipped() { return wheelFlipped; }
}
