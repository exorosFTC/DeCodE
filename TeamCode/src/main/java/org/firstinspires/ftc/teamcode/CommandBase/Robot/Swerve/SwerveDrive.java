package org.firstinspires.ftc.teamcode.CommandBase.Robot.Swerve;

import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoAngularD;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoAngularP;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.TeleOpAngularD;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.TeleOpAngularP;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.POSE;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.STRAFING_SLEW_RATE_LIMIT;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.TURNING_SLEW_RATE_LIMIT;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.TeleOpLimelightD;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.TeleOpLimelightP;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.TeleOpVelocityMultiplier;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.goalPosition;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.startPose;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.SystemConstants.autoOnBlue;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.SystemConstants.opModeType;
import static org.firstinspires.ftc.teamcode.CustomPathing.Math.MathFormulas.FindShortestPath;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandBase.Constants.SystemConstants;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Hardware;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.SystemBase;
import org.firstinspires.ftc.teamcode.CommandBase.Util.SensorsEx.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.CommandBase.Util.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.CustomPathing.Math.Geometry.Pose;

import java.util.List;

public class SwerveDrive extends SystemBase {
    private final Hardware hardware;
    public final SlewRateLimiter xLim, yLim, headLim;

    public List<SwerveModuleState> states;

    private final ElapsedTime timer;
    public final PIDController angularC, limelightC;
    public double targetHeading;

    private boolean lockHeadingToGoal = false;

    public SwerveModule leftFrontModule, rightFrontModule, rightBackModule, leftBackModule;
    public SwerveModule[] modules;



    public SwerveDrive(LinearOpMode opMode) {
        hardware = Hardware.getInstance(opMode);
        timer = new ElapsedTime();

        rightFrontModule = new SwerveModule(hardware.RightFront,
                                            hardware.RightFront_servo,
                                            new AbsoluteAnalogEncoder(hardware.RightFront_encoder).zero(1.132),
                                            0.088,
                                            0.045,
                                            0.05);
        leftFrontModule = new SwerveModule(hardware.LeftFront,
                                            hardware.LeftFront_servo,
                                            new AbsoluteAnalogEncoder(hardware.LeftFront_encoder).zero(0.079),
                                            0.068,
                                            0.04,
                                            0.05);
        leftBackModule = new SwerveModule(hardware.LeftBack,
                                            hardware.LeftBack_servo,
                                            new AbsoluteAnalogEncoder(hardware.LeftBack_encoder).zero(0.755),
                                            0.11,
                                            0.045,
                                            0.04);
        rightBackModule = new SwerveModule(hardware.RightBack,
                                            hardware.RightBack_servo,
                                            new AbsoluteAnalogEncoder(hardware.RightBack_encoder).zero(1.780),
                                            0.12,
                                            0.04,
                                            0.05);

        modules = new SwerveModule[]{rightFrontModule, leftFrontModule, leftBackModule, rightBackModule};
        states = SwerveKinematics.robot2wheel(new Pose(0, 0, 0));

        xLim = new SlewRateLimiter(STRAFING_SLEW_RATE_LIMIT);
        yLim = new SlewRateLimiter(STRAFING_SLEW_RATE_LIMIT);
        headLim = new SlewRateLimiter(TURNING_SLEW_RATE_LIMIT);


        angularC = new PIDController(TeleOpAngularP, 0, TeleOpAngularD);
        limelightC = new PIDController(TeleOpLimelightP, 0, TeleOpLimelightD);
        targetHeading = startPose.heading;
    }



    public void setModulePID(double p, double i, double d) {
        for (int a = 0; a < 4; a++)
            modules[a].setPID(p, i, d);
    }

    public void setHeadingPID(double p, double i, double d) { angularC.setPID(p, i, d); }

    public void setLimelightPID(double p, double i, double d) { limelightC.setPID(p, i, d); }



    public void update(Pose velocity) {
        /*if (autoOnBlue) velocity.negate();
        double power = velocity.hypot();

        if (!lockHeadingToGoal && opModeType == SystemConstants.OpMode.TELE_OP) {
            if (hardware.limelight.enabled) hardware.limelight.stop();

            //pid for skew correction
            if (Math.abs(velocity.heading) < 0.01 && timer.milliseconds() > 600) {
                angularC.setP(0);
                velocity.heading = angularC.calculate(FindShortestPath(POSE.heading, targetHeading));
            } else {
                if (Math.abs(velocity.heading) > 0.01)
                    timer.reset();
                targetHeading = POSE.heading;
            }
        }
        else if (lockHeadingToGoal && opModeType == SystemConstants.OpMode.TELE_OP) {
            if (!hardware.limelight.enabled) hardware.limelight.start();
            hardware.limelight.read();

            if (hardware.limelight.tagInSight()) {
                hardware.telemetry.addLine("Alignment: TAG");
                velocity.heading = limelightC.calculate(hardware.limelight.getCenterOffset());
            } else {
                hardware.telemetry.addLine("Alignment: ODOMETRY");
                targetHeading = Math.atan2(goalPosition.y - POSE.y, goalPosition.x - POSE.x);
                angularC.setP(TeleOpAngularP + power * TeleOpVelocityMultiplier);
                angularC.setPID(0.9, 0, 0);
                velocity.heading = angularC.calculate(FindShortestPath(POSE.heading, targetHeading));
            }
        }


        velocity = velocity.rotate_matrix(-POSE.heading + ((opModeType == SystemConstants.OpMode.TELE_OP) ? Math.toRadians(270) : 0));*/


        if (Math.abs(velocity.x) < 0.01 && Math.abs(velocity.y) < 0.01 && Math.abs(velocity.heading) < 0.01) {
            if (SwerveKinematics.isLockedX()) states = SwerveKinematics.robot2wheel(velocity);
            else { for (int i = 0; i < 4; i++) { states.get(i).setModuleVelocity(0); }}
        } else { SwerveKinematics.setLockedX(false); states = SwerveKinematics.robot2wheel(velocity); }

        /*boolean reached = true;
        for (int i = 0; i < 4; i++) {
            if (modules[i].error > Math.PI / 8) reached = false;
        }*/

        for (int i = 0; i < 4; i++) {
            if (i == 0 || i == 2) states.get(i).negateModuleVelocity();

            modules[i].setTargetState(states.get(i));
            modules[i].update();
        }
    }

    public void update(List<SwerveModuleState> states) {
        for (int i = 0; i < 4; i++) {
            if (i == 0 || i == 2) states.get(i).negateModuleVelocity();
            modules[i].setTargetState(states.get(i));
            modules[i].update();
        }
    }



    public void disable() {
        hardware.LeftFront.setMotorDisable();
        hardware.LeftBack.setMotorDisable();
        hardware.RightFront.setMotorDisable();
        hardware.RightBack.setMotorDisable();

        hardware.LeftFront_servo.setPower(0);
        hardware.LeftBack_servo.setPower(0);
        hardware.RightFront_servo.setPower(0);
        hardware.RightBack_servo.setPower(0);
    }

    public void setLockedX(boolean lockedX) { SwerveKinematics.setLockedX(lockedX); }

    public void lockHeadingToGoal(boolean lock) { lockHeadingToGoal = lock; }




    public void read() {
        leftFrontModule.read();
        leftBackModule.read();
        rightFrontModule.read();
        rightBackModule.read();
    }

    public void write() {
        leftFrontModule.write();
        leftBackModule.write();
        rightFrontModule.write();
        rightBackModule.write();
    }
}
