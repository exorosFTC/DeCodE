package org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.accelerationScalar;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.usingAcceleration;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.usingExponentialInput;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.opModeType;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.FrontUltrasonic;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.LeftBack;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.LeftFront;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.LeftUltrasonic;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.RightBack;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.RightFront;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.RightUltrasonic;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Util.SensorsEx.UltrasonicSensor;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

public class Drivetrain {
    private final Hardware hardware;
    private final LinearOpMode opMode;

    private double lastX = 0, lastY = 0, lastHead = 0;
    private double LF, RF, LB, RB;

    public final UltrasonicSensor left, right, front;

    public Drivetrain(LinearOpMode opMode) {
        hardware = Hardware.getInstance(opMode);

        hardware.motors.get(LeftBack).setDirection(DcMotorSimple.Direction.REVERSE);
        hardware.motors.get(LeftFront).setDirection(DcMotorSimple.Direction.REVERSE);

        left = new UltrasonicSensor(opMode, hardware.analog.get(LeftUltrasonic));
        right = new UltrasonicSensor(opMode, hardware.analog.get(RightUltrasonic));
        front = new UltrasonicSensor(opMode, hardware.analog.get(FrontUltrasonic));

        if (opModeType == Enums.OpMode.AUTONOMUS) {
            hardware.motors.get(LeftFront).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hardware.motors.get(LeftBack).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hardware.motors.get(RightFront).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hardware.motors.get(RightBack).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        this.opMode = opMode;
    }


    /**input velocity NEEDS to be clamped to [-1, 1]*/
    public void update(Pose velocity) {
        if (usingExponentialInput)
            velocity = exponential(velocity);

        if (usingAcceleration)
            velocity = accelerate(velocity);

        double x = velocity.x, y = velocity.y, head = velocity.heading;


        LF = x - y - head * TRACK_WIDTH;
        RF = x + y + head * TRACK_WIDTH;
        LB = x + y - head * TRACK_WIDTH;
        RB = x - y + head * TRACK_WIDTH;

        if (Math.abs(LF) < 0.05) LF = 0;
        if (Math.abs(RF) < 0.05) RF = 0;
        if (Math.abs(RB) < 0.05) RB = 0;
        if (Math.abs(LB) < 0.05) LB = 0;

        hardware.motors.get(LeftFront).setPower(LF);
        hardware.motors.get(LeftBack).setPower(LB);
        hardware.motors.get(RightFront).setPower(RF);
        hardware.motors.get(RightBack).setPower(RB);

    }

    public void update(double lf, double lb, double rf, double rb) {
        hardware.motors.get(LeftFront).setPower(lf);
        hardware.motors.get(LeftBack).setPower(lb);
        hardware.motors.get(RightFront).setPower(rf);
        hardware.motors.get(RightBack).setPower(rb);
    }



    private Pose exponential(Pose velocity) {
        return new Pose(Math.pow(velocity.x, 3),
                Math.pow(velocity.y, 3),
                Math.pow(velocity.heading, 3));
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

}
