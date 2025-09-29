package org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Drivetrain.Swerve;

import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.accelerationScalar;
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Util.SensorsEx.AbsoluteAnalogEncoderEx;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

import java.util.List;

public class SwerveDrive extends SwerveKinematics {
    private final Hardware hardware;
    private final LinearOpMode opMode;


    private double lastX = 0, lastY = 0, lastHead = 0;
    public SwerveModule rightFrontModule, leftFrontModule, leftBackModule, rightBackModule;
    public SwerveModule[] modules;



    public SwerveDrive(LinearOpMode opMode) {
        hardware = Hardware.getInstance(opMode);

        rightFrontModule = new SwerveModule(hardware.motors.get(RightFront),
                                            hardware.CRservos.get(RightFront_servo),
                                            new AbsoluteAnalogEncoderEx(hardware.analog.get(RightFront_encoder)));
        leftFrontModule = new SwerveModule(hardware.motors.get(LeftFront),
                                            hardware.CRservos.get(LeftFront_servo),
                                            new AbsoluteAnalogEncoderEx(hardware.analog.get(LeftFront_encoder)));
        leftBackModule = new SwerveModule(hardware.motors.get(LeftBack),
                                            hardware.CRservos.get(LeftBack_servo),
                                            new AbsoluteAnalogEncoderEx(hardware.analog.get(LeftBack_encoder)));
        rightBackModule = new SwerveModule(hardware.motors.get(RightBack),
                                            hardware.CRservos.get(RightBack_servo),
                                            new AbsoluteAnalogEncoderEx(hardware.analog.get(RightBack_encoder)));

        this.opMode = opMode;

        modules = new SwerveModule[]{rightFrontModule, leftFrontModule, leftBackModule, rightBackModule};

        //TODO: reverse directions if necessary
        //setDirection(Arrays.asList(
        // DcMotorSimple.Direction.REVERSE,
        // DcMotorSimple.Direction.FORWARD,
        // DcMotorSimple.Direction.FORWARD,
        // DcMotorSimple.Direction.REVERSE));
    }



    public void setDirection(List<DcMotorSimple.Direction> directions) {
        for (int i = 0; i < 4; i++)
            modules[i].setDirection(directions.get(i));
    }



    public void update(Pose velocity) {
        // (usingFieldCentric)
        //    vel = new velocity.rotate_matrix(-localizer.getAngle());
        // else vel = new Pose(joystick_x, joystick_y, joystick_head);

        if (velocity.x < 0.001 && velocity.y < 0.001 & velocity.heading < 0.001)
            super.setLocked(true);
        else super.setLocked(false);

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



    private void disable() {
        hardware.motors.get(LeftFront).setMotorDisable();
        hardware.motors.get(LeftBack).setMotorDisable();
        hardware.motors.get(RightFront).setMotorDisable();
        hardware.motors.get(RightBack).setMotorDisable();

        // closes all servos because they are all in on the same hub
        hardware.CRservos.get(LeftFront_servo).close();
    }
}
