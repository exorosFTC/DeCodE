package org.firstinspires.ftc.teamcode.CustomPathing;

import static com.qualcomm.robotcore.util.Range.clip;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoAngularD;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoAngularP;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoAngularVelocityMultiplier;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoLinearDx;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoLinearDy;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoLinearPx;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoLinearPy;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.POSE;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.VELOCITY;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.startPose;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.useVelocityTimeout;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.SystemConstants.lastValidRandomization;
import static org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.Subsystems.Indexer.elements;
import static org.firstinspires.ftc.teamcode.CustomPathing.Math.MathFormulas.FindShortestPath;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.ScoringSystem;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Hardware;
import org.firstinspires.ftc.teamcode.CommandBase.Util.SensorsEx.HubBulkRead;
import org.firstinspires.ftc.teamcode.CommandBase.Util.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.CustomPathing.Math.Geometry.Point;
import org.firstinspires.ftc.teamcode.CustomPathing.Math.Geometry.Pose;


import java.util.concurrent.TimeUnit;
import java.util.function.BooleanSupplier;

import kotlin.collections.SlidingWindowKt;

public class AutoDrive {
    private final Hardware hardware;
    private final SwerveDrive swerve;
    private final ScoringSystem system;

    private final LinearOpMode opMode;
    private final ElapsedTime waitTimer;

    private Thread driveThread, systemThread;
    public final PIDController linearCx, linearCy, angularC;

    private double busyThresholdLinear = 0.9,
            busyThresholdAngular = Math.toRadians(3);

    private double failSafeTimeMs = Double.POSITIVE_INFINITY;
    private final ElapsedTime failSafeTimer = new ElapsedTime();
    private final ElapsedTime velocityTimeoutTimer = new ElapsedTime();

    private Pose target = new Pose();
    private Pose driveVector = new Pose();

    private boolean usingFailSafe = false;
    private double maxDistance = 0;
    private double currentDistance = 0;
    private double currentVelocity = 0;
    private double maxSpeed = 0.8;

    public static double kS_angular = 0.06;
    public static SlewRateLimiter limX, limY;






    public AutoDrive(LinearOpMode opMode, SwerveDrive swerve, ScoringSystem system, Pose startPose) {
        this.swerve = swerve;
        this.system = system;
        POSE = startPose;

        hardware = Hardware.getInstance(opMode);
        setPose(POSE);

        linearCx = new PIDController(AutoLinearPx, 0, AutoLinearDx);
        linearCy = new PIDController(AutoLinearPy, 0, AutoLinearDy);
        angularC = new PIDController(AutoAngularP, 0, AutoAngularD);

        limX = new SlewRateLimiter(2);
        limY = new SlewRateLimiter(2);

        this.opMode = opMode;

        waitTimer = new ElapsedTime();
        startDriveThread();
    }

    private void startDriveThread() {
        driveThread = new Thread(() -> {
            opMode.waitForStart();

            while (opMode.opModeIsActive()) {
                swerve.read();

                hardware.telemetry.addData("x", POSE.x);
                hardware.telemetry.addData("y", POSE.y);
                hardware.telemetry.addData("head", Math.toDegrees(POSE.heading));
                hardware.telemetry.addData("isBusy", isBusy());
                hardware.telemetry.addData("randomization", lastValidRandomization);
                hardware.telemetry.addData("artifacts", elements.toString());

                hardware.telemetry.update();

                if (usingFailSafe && isBusy() && failSafeTimer.time(TimeUnit.MILLISECONDS) > failSafeTimeMs)
                    driveTo(POSE,  30, 30);

                updateDriveVector();
                swerve.update(driveVector);
                swerve.write();

            }

            swerve.update(new Pose());
            swerve.write();
            swerve.disable();

            startPose = POSE;
        });
        systemThread = new Thread(() -> {
            opMode.waitForStart();

            while (opMode.opModeIsActive()) {
                hardware.bulk.clearCache(HubBulkRead.Hubs.ALL);
                hardware.localizer.update();
                hardware.readBattery();
                system.read();

                system.shooter.update();
                system.shooter.write();

                Thread.yield();
            }

            system.shooter.off();
            system.intake.off();
            system.indexer.off();
            system.shooter.disable();
        });

        driveThread.start();
        systemThread.start();
    }





    public AutoDrive driveTo(Pose pose, double speed) {
        return driveTo(pose, speed, Double.POSITIVE_INFINITY);
    }

    public AutoDrive driveTo(Pose pose, double speed, double ms) {
        this.failSafeTimeMs = ms;
        this.usingFailSafe = ms != Double.POSITIVE_INFINITY;

        this.target = new Pose(pose.x, pose.y, normalizeAngleRad(pose.heading));
        this.maxDistance = pose.hypot(POSE);
        this.maxSpeed = speed;

        failSafeTimer.reset();
        return this;
    }







    public AutoDrive waitDrive(double threshold) { return waitDrive(opMode::idle, threshold, false); }

    public AutoDrive waitDrive(double threshold, boolean useHeading) { return waitDrive(opMode::idle, threshold, useHeading); }

    public AutoDrive waitDrive(Runnable inLoop, double threshold, boolean useHeading) {
        this.busyThresholdLinear = threshold;
        updateDriveVector();

        try { Thread.sleep(10); } catch (InterruptedException e) {}
        velocityTimeoutTimer.reset();

        while ((useHeading ? isBusy() : Math.abs(currentDistance) > Math.abs(maxDistance * (1 - busyThresholdLinear))) && opMode.opModeIsActive() && (!useVelocityTimeout || velocityTimeoutTimer.milliseconds() < 800)) {
            if (useVelocityTimeout && currentVelocity > 8)
                velocityTimeoutTimer.reset();
            inLoop.run();
        }

        return this;
    }




    public AutoDrive waitMs(double ms) {
        return waitMs(ms, opMode::idle);
    }

    public AutoDrive waitMs(double ms, Runnable action) {
        waitTimer.reset();
        while (waitTimer.time(TimeUnit.MILLISECONDS) <= ms && opMode.opModeIsActive()) { action.run(); }
        return this;
    }



    public AutoDrive waitAction(BooleanSupplier action) {
        while (!action.getAsBoolean() && opMode.opModeIsActive()) { opMode.idle(); }
        return this;
    }

    public AutoDrive waitAction(BooleanSupplier action, Runnable inLoop) {
        while (!action.getAsBoolean() && opMode.opModeIsActive()) { inLoop.run(); }
        return this;
    }






    public AutoDrive moveSystem(Runnable action) {
        if (action != null) action.run();
        return this;
    }

    public AutoDrive setBusyThresholdLinear(double pow) {
        this.busyThresholdLinear = pow;
        return this;
    }

    public AutoDrive setBusyThresholdAngular(double rad) {
        this.busyThresholdAngular = rad;
        return this;
    }

    public AutoDrive setPose(Pose pose) {
        try { Thread.sleep(150); } catch (InterruptedException e) {}
        hardware.localizer.setPositionEstimate(pose);
        try { Thread.sleep(150); } catch (InterruptedException e) {}

        return this;
    }





    private void updateDriveVector() {
        currentDistance = target.hypot(POSE);
        currentVelocity = VELOCITY.hypot();

        double angularVelocity = angularC.calculate(FindShortestPath(POSE.heading, target.heading));

        driveVector = new Pose(
                limX.calculate(Range.clip(linearCx.calculate(POSE.x, target.x), -maxSpeed, maxSpeed)),
                limY.calculate(Range.clip(linearCy.calculate(POSE.y, target.y), -maxSpeed, maxSpeed)),
                Range.clip(Math.abs(currentVelocity) > 4 ? angularVelocity : angularVelocity + Math.signum(angularVelocity) * kS_angular, -1, 1)
        ).multiplyBy(11.0 / hardware.batteryVoltage);
    }




    private double normalizeAngleRad(double angle) {
        if (angle < 0) return angle + 2 * Math.PI;
        return angle;
    }

    public boolean isBusy() {
        return Math.abs(currentDistance) > Math.abs(maxDistance * (1 - busyThresholdLinear))
                ||
                Math.abs(target.heading - POSE.heading) > busyThresholdAngular;
    }




}