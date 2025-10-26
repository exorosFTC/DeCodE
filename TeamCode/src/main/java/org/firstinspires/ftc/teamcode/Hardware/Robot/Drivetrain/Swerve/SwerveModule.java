package org.firstinspires.ftc.teamcode.Hardware.Robot.Drivetrain.Swerve;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.K_STATIC;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.swerveD;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.swerveP;

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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.Util.SensorsEx.AbsoluteAnalogEncoder;


public class SwerveModule {
    private final DcMotorEx motor;
    private final CRServo servo;
    private final AbsoluteAnalogEncoder encoder;
    private final PIDFController controller = new PIDFController(swerveP, 0, swerveD, 0);

    private boolean wheelFlipped = false;

    private SwerveModuleState currentState = new SwerveModuleState(0, 0);
    private SwerveModuleState targetState = new SwerveModuleState(0, 0);



    public SwerveModule(HardwareMap hardwareMap, String motor_name, String servo_name, String encoder_name) {
        motor = hardwareMap.get(DcMotorEx.class, motor_name);
        servo = hardwareMap.get(CRServo.class, servo_name);
        encoder = new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, encoder_name));

        initialize();
    }

    public SwerveModule(DcMotorEx motor, CRServo servo, AbsoluteAnalogEncoder encoder) {
        this.motor = motor;
        this.servo = servo;
        this.encoder = encoder;

        // uncomment if you didn't initialize them properly before
        // initialize();
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

        if (currentState.getModuleVelocity() != targetState.getModuleVelocity()) {
            motor.setPower(wheelFlipped ? -power : power);
            targetState.setModuleVelocity(currentState.getModuleVelocity());
        }
    }

    public double getTargetRotation() {
        return normalizeRadians(targetState.getModuleAngle() - Math.PI);
    }

    public double getModuleRotation() {
        return normalizeRadians(currentState.getModuleAngle() - Math.PI);
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
}
