package org.firstinspires.ftc.teamcode.Hardware.Robot.Swerve;

import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.AngularD;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.AngularP;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.POSE;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.goalPosition;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.startPose;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.usingFieldCentric;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.SystemConstants.opModeType;
import static org.firstinspires.ftc.teamcode.Pathing.Math.MathFormulas.FindShortestPath;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Constants.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot.SystemBase;
import org.firstinspires.ftc.teamcode.Hardware.Util.SensorsEx.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

import java.util.List;

public class SwerveDrive extends SystemBase {
    private final Hardware hardware;

    public List<SwerveModuleState> states;

    private final ElapsedTime timer;
    public final PIDController angularC;
    public double targetHeading;

    private boolean lockHeadingToGoal = false;

    public SwerveModule leftFrontModule, rightFrontModule, rightBackModule, leftBackModule;
    public SwerveModule[] modules;



    public SwerveDrive(LinearOpMode opMode) {
        hardware = Hardware.getInstance(opMode);
        timer = new ElapsedTime();

        rightFrontModule = new SwerveModule(hardware.RightFront,
                                            hardware.RightFront_servo,
                                            new AbsoluteAnalogEncoder(hardware.RightFront_encoder).zero(-2.861));
        leftFrontModule = new SwerveModule(hardware.LeftFront,
                                            hardware.LeftFront_servo,
                                            new AbsoluteAnalogEncoder(hardware.LeftFront_encoder).zero(-3.0335));
        leftBackModule = new SwerveModule(hardware.LeftBack,
                                            hardware.LeftBack_servo,
                                            new AbsoluteAnalogEncoder(hardware.LeftBack_encoder).zero(2.926));
        rightBackModule = new SwerveModule(hardware.RightBack,
                                            hardware.RightBack_servo,
                                            new AbsoluteAnalogEncoder(hardware.RightBack_encoder).zero(0.763));

        modules = new SwerveModule[]{rightFrontModule, leftFrontModule, leftBackModule, rightBackModule};
        states = SwerveKinematics.robot2wheel(new Pose(0, 0, 0));


        angularC = new PIDController(AngularP, 0, AngularD);
        targetHeading = startPose.heading;
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
            targetHeading = Math.atan2(goalPosition.y - POSE.y, goalPosition.x - POSE.x);
            velocity.heading = -angularC.calculate(FindShortestPath(POSE.heading, targetHeading));
        }

        if (usingFieldCentric || opModeType == Enums.OpMode.AUTONOMUS)
            velocity = velocity.rotate_matrix(-POSE.heading + startPose.heading);

        if (Math.abs(velocity.x) < 0.01 && Math.abs(velocity.y) < 0.01 && Math.abs(velocity.heading) < 0.01) {
            for (int i = 0; i < 4; i++) {
                states.get(i).setModuleVelocity(0);
            }
        } else states = SwerveKinematics.robot2wheel(velocity);

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



    public void disable() {
        hardware.LeftFront.setMotorDisable();
        hardware.LeftBack.setMotorDisable();
        hardware.RightFront.setMotorDisable();
        hardware.RightBack.setMotorDisable();
    }

    public void setLockedX(boolean lockedX) { SwerveKinematics.setLocked(lockedX); }

    public void lockHeadingToGoal(boolean lock) { lockHeadingToGoal = lock; }



    @Override
    public void read() {
        leftFrontModule.read();
        leftBackModule.read();
        rightFrontModule.read();
        rightBackModule.read();
    }

    @Override
    public void write() {
        leftFrontModule.write();
        leftBackModule.write();
        rightFrontModule.write();
        rightBackModule.write();
    }
}
