package org.firstinspires.ftc.teamcode.CommandBase.Robot.Swerve;

import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoAngularVelocityMultiplier;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.TELE_OP_STRAFING_SLEW_RATE_LIMIT;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.TELE_OP_TURNING_SLEW_RATE_LIMIT;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.TeleOpAngularD;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.TeleOpAngularP;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.POSE;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AUTO_STRAFING_SLEW_RATE_LIMIT;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AUTO_TURNING_SLEW_RATE_LIMIT;
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
                                            new AbsoluteAnalogEncoder(hardware.RightFront_encoder).zero(-1.97),
                                            0.03,
                                            0.05);
        leftFrontModule = new SwerveModule(hardware.LeftFront,
                                            hardware.LeftFront_servo,
                                            new AbsoluteAnalogEncoder(hardware.LeftFront_encoder).zero(0.1),
                                            0.03,
                                            0.05);
        leftBackModule = new SwerveModule(hardware.LeftBack,
                                            hardware.LeftBack_servo,
                                            new AbsoluteAnalogEncoder(hardware.LeftBack_encoder).zero(-2.38),
                                            0.03,
                                            0.05);
        rightBackModule = new SwerveModule(hardware.RightBack,
                                            hardware.RightBack_servo,
                                            new AbsoluteAnalogEncoder(hardware.RightBack_encoder).zero(1.75),
                                            0.03,
                                            0.05);

        modules = new SwerveModule[]{rightFrontModule, leftFrontModule, leftBackModule, rightBackModule};
        states = SwerveKinematics.robot2wheel(new Pose(0, 0, 0));

        xLim = new SlewRateLimiter(opModeType == SystemConstants.OpMode.AUTONOMOUS ? AUTO_STRAFING_SLEW_RATE_LIMIT : TELE_OP_STRAFING_SLEW_RATE_LIMIT);
        yLim = new SlewRateLimiter(opModeType == SystemConstants.OpMode.AUTONOMOUS ? AUTO_STRAFING_SLEW_RATE_LIMIT : TELE_OP_STRAFING_SLEW_RATE_LIMIT);
        headLim = new SlewRateLimiter(opModeType == SystemConstants.OpMode.AUTONOMOUS ? AUTO_TURNING_SLEW_RATE_LIMIT : TELE_OP_TURNING_SLEW_RATE_LIMIT);

        angularC = new PIDController(TeleOpAngularP, 0, TeleOpAngularD);
        limelightC = new PIDController(TeleOpLimelightP, 0, TeleOpLimelightD);
        targetHeading = startPose.heading;

        super.on();
    }



    public void setModulePID(double p, double i, double d) {
        for (int a = 0; a < 4; a++)
            modules[a].setPID(p, i, d);
    }

    public void setModuleKs(double kS) {
        for (int a = 0; a < 4; a++)
            modules[a].setAngleKS(kS);
    }

    public void setHeadingPID(double p, double i, double d) { angularC.setPID(p, i, d); }

    public void setLimelightPID(double p, double i, double d) { limelightC.setPID(p, i, d); }



    public void update(Pose velocity) {
        // stop updating the swerve if disabled
        if (!on) return;

        // correct joystick input for blue side in teleop
        if (autoOnBlue && opModeType == SystemConstants.OpMode.TELE_OP) velocity.negate();

        // raw linear power for angular correction
        double power = velocity.hypot();


        // keep heading & use joystick input when not aligning to the goal
        if (!lockHeadingToGoal) {
            //if (hardware.limelight.enabled) hardware.limelight.stop();

            // pid for skew correction
            if (Math.abs(velocity.heading) < 0.01 && timer.milliseconds() > 600) {
                angularC.setP(power * (opModeType == SystemConstants.OpMode.TELE_OP ? TeleOpVelocityMultiplier : AutoAngularVelocityMultiplier));
                velocity.heading = angularC.calculate(FindShortestPath(POSE.heading, targetHeading));
            } else {
                if (Math.abs(velocity.heading) > 0.01)
                    timer.reset();
                targetHeading = POSE.heading;
            }
        }
        // ignore joystick input & align the heading to the goal
        else {
            /*if (!hardware.limelight.enabled) hardware.limelight.start();
            hardware.limelight.read();

            if (hardware.limelight.tagInSight()) {
                hardware.telemetry.addLine("Alignment: TAG");
                velocity.heading = limelightC.calculate(hardware.limelight.getCenterOffset());
            } else {*/
                //hardware.telemetry.addLine("Alignment: ODOMETRY");
                targetHeading = Math.atan2(goalPosition.y - POSE.y, goalPosition.x - POSE.x);
                angularC.setPID(0.9, 0, 0);
                velocity.heading = angularC.calculate(FindShortestPath(POSE.heading, targetHeading));
            //}
        }


        // convert to field centric anytime
        velocity = velocity.rotate_matrix(-POSE.heading + ((opModeType == SystemConstants.OpMode.TELE_OP) ? Math.toRadians(270) : 0));

        // apply slew rate limiting
        velocity = new Pose(xLim.calculate(velocity.x),
                            yLim.calculate(velocity.y),
                            headLim.calculate(velocity.heading));


        // check for joystick deadband
        if (Math.abs(velocity.x) < 0.01 && Math.abs(velocity.y) < 0.01 && Math.abs(velocity.heading) < 0.01) {
            if (SwerveKinematics.isLockedX()) states = SwerveKinematics.robot2wheel(velocity);
            else { for (int i = 0; i < 4; i++) { states.get(i).setModuleVelocity(0); }}
        } else { SwerveKinematics.setLockedX(false); states = SwerveKinematics.robot2wheel(velocity); }

        // update the module
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
        this.on = false;

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
