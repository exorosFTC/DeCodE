package org.firstinspires.ftc.teamcode.CommandBase.Robot.Swerve;

import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.TeleOpAngularD;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.TeleOpAngularP;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.POSE;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.VELOCITY;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.VEL_X_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.VEL_Y_MULTIPLIER;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.goalPosition;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.goalPositionBlue;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.goalPositionRed;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.startPose;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.SystemConstants.autoOnBlue;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.SystemConstants.opModeType;
import static org.firstinspires.ftc.teamcode.CustomPathing.Math.MathFormulas.FindShortestPath;
import static org.firstinspires.ftc.teamcode.CustomPathing.Math.MathFormulas.midPoint;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.CommandBase.Constants.SystemConstants;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Hardware;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.SystemBase;
import org.firstinspires.ftc.teamcode.CommandBase.Util.SensorsEx.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.CommandBase.Util.SensorsEx.LimelightEx;
import org.firstinspires.ftc.teamcode.CustomPathing.Math.Geometry.Point;
import org.firstinspires.ftc.teamcode.CustomPathing.Math.Geometry.Pose;

import java.util.List;

public class SwerveDrive extends SystemBase {
    private final Hardware hardware;

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
                                            new AbsoluteAnalogEncoder(hardware.RightFront_encoder).zero(2.43),
                                            0.03,
                                            0.05);
        leftFrontModule = new SwerveModule(hardware.LeftFront,
                                            hardware.LeftFront_servo,
                                            new AbsoluteAnalogEncoder(hardware.LeftFront_encoder).zero(2.05),
                                            0.03,
                                            0.05);
        leftBackModule = new SwerveModule(hardware.LeftBack,
                                            hardware.LeftBack_servo,
                                            new AbsoluteAnalogEncoder(hardware.LeftBack_encoder).zero(-1.61),
                                            0.03,
                                            0.05);
        rightBackModule = new SwerveModule(hardware.RightBack,
                                            hardware.RightBack_servo,
                                            new AbsoluteAnalogEncoder(hardware.RightBack_encoder).zero(-2.54),
                                            0.03,
                                            0.05);

        modules = new SwerveModule[]{rightFrontModule, leftFrontModule, leftBackModule, rightBackModule};
        states = SwerveKinematics.robot2wheel(new Pose(0, 0, 0));

        angularC = new PIDController(TeleOpAngularP, 0, TeleOpAngularD);
        limelightC = new PIDController(0, 0, 0);
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

        updateShootOnTheMove();

        // lock to goal logic
        if (lockHeadingToGoal) {
            if (!hardware.limelight.enabled) { hardware.limelight.start(); hardware.limelight.setPipeline(autoOnBlue ? LimelightEx.Pipeline.BLUE_GOAL : LimelightEx.Pipeline.RED_GOAL); }
            hardware.limelight.read();

            if (hardware.limelight.tagInSight()) {
                hardware.telemetry.addLine("Alignment: LIMELIGHT");
                double offset = hardware.limelight.getCenterOffset();

                if (Math.abs(offset) > DriveConstants.llThreshold) limelightC.setPID(DriveConstants.llFarP, 0, DriveConstants.llCloseD);
                else limelightC.setP(DriveConstants.llCloseP);

                velocity.heading = limelightC.calculate(offset) * 10 / hardware.batteryVoltage;
            } else {
                hardware.telemetry.addLine("Alignment: ODOMETRY");

                velocity.heading = angularC.calculate(FindShortestPath(POSE.heading, targetHeading));
                velocity.heading += Math.signum(velocity.heading) * Math.abs(VELOCITY.hypot()) > 3 ? 0 : 0.1;
            }
        } else if (hardware.limelight.enabled) hardware.limelight.stop();
        else hardware.telemetry.addLine("Alignment: OFF");


        // convert to field centric anytime
        velocity = velocity.rotate_matrix(-POSE.heading + ((opModeType == SystemConstants.OpMode.TELE_OP) ? Math.toRadians(270) : 0));

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

    public void updateShootOnTheMove() {
        // shift goal backward by where robot will move during release delay
        double virtualGoalX = autoOnBlue ? goalPositionBlue.x : goalPositionRed.x - VELOCITY.x * VEL_X_MULTIPLIER;
        double virtualGoalY = autoOnBlue ? goalPositionBlue.y : goalPositionRed.y - VELOCITY.y * VEL_Y_MULTIPLIER;

        goalPosition = new Point(virtualGoalX, virtualGoalY);
        targetHeading = Math.atan2(virtualGoalY - POSE.y, virtualGoalX - POSE.x);
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
