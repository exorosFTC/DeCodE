package org.firstinspires.ftc.teamcode.Hardware.Robot.Drivetrain.Swerve;

import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.AngularD;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.AngularP;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.POSE;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.accelerationScalar;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.blueGoalPosition;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.redGoalPosition;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.usingAcceleration;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.usingExponentialInput;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.usingFieldCentric;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.HardwareNames.LeftBack;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.HardwareNames.LeftBack_encoder;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.HardwareNames.LeftBack_servo;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.HardwareNames.LeftFront;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.HardwareNames.LeftFront_encoder;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.HardwareNames.LeftFront_servo;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.HardwareNames.RightBack;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.HardwareNames.RightBack_encoder;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.HardwareNames.RightBack_servo;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.HardwareNames.RightFront;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.HardwareNames.RightFront_encoder;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.HardwareNames.RightFront_servo;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.SystemConstants.autoOnBlue;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.SystemConstants.opModeType;
import static org.firstinspires.ftc.teamcode.Pathing.Math.MathFormulas.FindShortestPath;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.Constants.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Util.SensorsEx.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.Pathing.Localizer.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.Pathing.Math.Point;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

import java.util.List;

public class SwerveDrive extends SwerveKinematics {
    private final Hardware hardware;

    public final PinpointLocalizer localizer;

    public final PIDController angularC;
    public double targetHeading = 0;
    public ElapsedTime timer;

    private boolean lockHeadingToGoal = false;

    private double lastX = 0, lastY = 0, lastHead = 0;
    public SwerveModule leftFrontModule, rightFrontModule, rightBackModule, leftBackModule;
    public SwerveModule[] modules;



    public SwerveDrive(LinearOpMode opMode) {
        hardware = Hardware.getInstance(opMode);


        rightFrontModule = new SwerveModule(hardware.motors.get(RightFront),
                                            hardware.CRservos.get(RightFront_servo),
                                            new AbsoluteAnalogEncoder(hardware.analog.get(RightFront_encoder)).zero(-2.861));
        leftFrontModule = new SwerveModule(hardware.motors.get(LeftFront),
                                            hardware.CRservos.get(LeftFront_servo),
                                            new AbsoluteAnalogEncoder(hardware.analog.get(LeftFront_encoder)).zero(-3.0335));
        leftBackModule = new SwerveModule(hardware.motors.get(LeftBack),
                                            hardware.CRservos.get(LeftBack_servo),
                                            new AbsoluteAnalogEncoder(hardware.analog.get(LeftBack_encoder)).zero(2.926));
        rightBackModule = new SwerveModule(hardware.motors.get(RightBack),
                                            hardware.CRservos.get(RightBack_servo),
                                            new AbsoluteAnalogEncoder(hardware.analog.get(RightBack_encoder)).zero(0.763));

        modules = new SwerveModule[]{rightFrontModule, leftFrontModule, leftBackModule, rightBackModule};


        localizer = new PinpointLocalizer(opMode.hardwareMap);
        angularC = new PIDController(AngularP, 0, AngularD);
        timer = new ElapsedTime();
    }



    public void setDirection(List<DcMotorSimple.Direction> directions) {
        for (int i = 0; i < 4; i++)
            modules[i].setDirection(directions.get(i));
    }

    public void setModulePID(double p, double i, double d) {
        for (int a = 0; a < 4; a++)
            modules[a].setPID(p, i, d);
    }

    public void setHeadingPID(double p, double i, double d) {
        angularC.setPID(p, i, d);
    }



    public void update(Pose velocity) {
        localizer.update();

        if (opModeType == Enums.OpMode.TELE_OP) {
            if (usingExponentialInput)
                velocity = exponential(velocity);
            if (usingAcceleration)
                velocity = accelerate(velocity);


            if (!lockHeadingToGoal) {
                //pid for skew correction
                if (Math.abs(velocity.heading) < 0.01 && timer.milliseconds() > 600) {
                    velocity.heading = -angularC.calculate(FindShortestPath(POSE.heading, targetHeading));
                } else {
                    if (Math.abs(velocity.heading) > 0.01)
                        timer.reset();
                    targetHeading = POSE.heading;
                }
            } else {
                Point target;

                if (autoOnBlue) target = blueGoalPosition;
                else target = redGoalPosition;

                targetHeading = Math.atan2(target.y - POSE.y, target.x - POSE.x);
                velocity.heading = -angularC.calculate(FindShortestPath(POSE.heading, targetHeading));
            }
        }

        if (usingFieldCentric || opModeType == Enums.OpMode.AUTONOMUS)
            velocity = velocity.rotate_matrix(-POSE.heading);

        if (Math.abs(velocity.x) < 0.001 &&  Math.abs(velocity.y) < 0.001 && Math.abs(velocity.heading) < 0.001)
            setLocked(true);
        else setLocked(false);

        List<SwerveModuleState> states = super.robot2wheel(velocity);

        for (int i = 0; i < 4; i++) {
            modules[i].setTargetState(states.get(i));
            modules[i].update();
        }
    }

    public void update(List<SwerveModuleState> states) {
        for (int i = 0; i < 4; i++) {
            modules[i].setTargetState(states.get(i));
            modules[i].update();
        }
    }



    private Pose exponential(Pose velocity) {
        return new Pose(
                velocity.x * velocity.x * velocity.x,
                velocity.y * velocity.y * velocity.y,
                velocity.heading * velocity.heading * velocity.heading);
    }

    private Pose accelerate(Pose velocity) {
        if (velocity.x == 0) lastX = 0;
        else if (Math.abs(velocity.x - lastX) > accelerationScalar) {
            lastX += Math.signum(velocity.x) * accelerationScalar;
            velocity.x = lastX;
        }

        if (velocity.y == 0) lastY = 0;
        else if (Math.abs(velocity.y - lastY) > accelerationScalar) {
            lastY += Math.signum(velocity.y) * accelerationScalar;
            velocity.y = lastY;
        }

        if (velocity.heading == 0) lastHead = 0;
        else if (Math.abs(velocity.heading - lastHead) > accelerationScalar) {
            lastHead += Math.signum(velocity.heading) * accelerationScalar;
            velocity.heading = lastHead;
        }

        return velocity;
    }



    public void disable() {
        hardware.motors.get(LeftFront).setMotorDisable();
        hardware.motors.get(LeftBack).setMotorDisable();
        hardware.motors.get(RightFront).setMotorDisable();
        hardware.motors.get(RightBack).setMotorDisable();
    }

    public void setLocked(boolean locked) { super.setLocked(locked); }

    public void setLockedX(boolean lockedX) { super.setLockedX(lockedX); }

    public void lockHeadingToGoal(boolean lock) { lockHeadingToGoal = lock; }

    public boolean isHeadingLockedToGoal() { return lockHeadingToGoal; }
}
