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
import static org.firstinspires.ftc.teamcode.CustomPathing.Math.MathFormulas.FindShortestPath;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.ScoringSystem;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Hardware;
import org.firstinspires.ftc.teamcode.CommandBase.Util.SensorsEx.HubBulkRead;
import org.firstinspires.ftc.teamcode.CustomPathing.Math.Geometry.Point;
import org.firstinspires.ftc.teamcode.CustomPathing.Math.Geometry.Pose;


import java.util.concurrent.TimeUnit;
import java.util.function.BooleanSupplier;

public class AutoDrive {
    private final Hardware hardware;
    private final SwerveDrive swerve;
    private final ScoringSystem system;

    private final LinearOpMode opMode;
    private final ElapsedTime waitTimer;

    private Thread driveThread, systemThread;
    public final PIDController linearCx, linearCy, angularC;

    private double busyThresholdLinear = 0.9,
                   busyThresholdAngular = Math.toRadians(6);

    private double failSafeTimeMs = Double.POSITIVE_INFINITY;
    private final ElapsedTime failSafeTimer = new ElapsedTime();

    private Pose target = new Pose();
    private Pose driveVector = new Pose();

    private boolean usingFailSafe = false;
    private double maxDistance = 0;
    private double currentDistance = 0;

    private double m, n;
    private double a, b, c;
    private double radius = 30;

    private Point followPoint = new Point();





    public AutoDrive(LinearOpMode opMode, SwerveDrive swerve, ScoringSystem system, Pose startPose) {
        this.swerve = swerve;
        this.system = system;
        POSE = startPose;

        hardware = Hardware.getInstance(opMode);
        setPose(POSE);

        linearCx = new PIDController(AutoLinearPx, 0, AutoLinearDx);
        linearCy = new PIDController(AutoLinearPy, 0, AutoLinearDy);
        angularC = new PIDController(AutoAngularP, 0, AutoAngularD);

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
                hardware.telemetry.addData("target x", followPoint.x);
                hardware.telemetry.addData("target y", followPoint.y);
                hardware.telemetry.addData("target head", Math.toDegrees(target.heading));
                hardware.telemetry.addData("isBusy", isBusy());
                hardware.telemetry.addData("current", Math.abs(currentDistance));
                hardware.telemetry.addData("max", Math.abs(maxDistance * (1 - busyThresholdLinear)));
                hardware.telemetry.update();

                if (usingFailSafe && isBusy() && failSafeTimer.time(TimeUnit.MILLISECONDS) > failSafeTimeMs)
                    driveTo(POSE, 30);

                updateDriveVector();
                swerve.update(driveVector);
                swerve.write();

            }
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
                system.write();

                Thread.yield();
            }
        });

        driveThread.start();
        systemThread.start();
    }





    public AutoDrive driveTo(Pose pose, double radius) {
        return driveTo(pose, radius, Double.POSITIVE_INFINITY);
    }

    public AutoDrive driveTo(Pose pose, double radius, double ms) {
        this.radius = radius;
        this.failSafeTimeMs = ms;
        this.usingFailSafe = ms != Double.POSITIVE_INFINITY;
        this.target = new Pose(pose.x, pose.y, normalizeAngleRad(pose.heading));

        updateLineEquation();
        updateDistance();

        failSafeTimer.reset();

        return this;
    }







    public AutoDrive waitDrive(double threshold) { return waitDrive(opMode::idle, threshold, false); }

    public AutoDrive waitDrive(double threshold, boolean useHeading) { return waitDrive(opMode::idle, threshold, useHeading); }

    public AutoDrive waitDrive(Runnable inLoop, double threshold, boolean useHeading) {
        this.busyThresholdLinear = threshold;

        updateDriveVector();
        try { Thread.sleep(10); } catch (InterruptedException e) {}
        while ((useHeading ? isBusy() : Math.abs(currentDistance) > Math.abs(maxDistance * (1 - busyThresholdLinear))) && opMode.opModeIsActive()) { inLoop.run(); }

        target = POSE;
        return this;
    }




    public AutoDrive waitMs(double ms) {
        waitTimer.reset();
        while (waitTimer.time(TimeUnit.MILLISECONDS) <= ms && opMode.opModeIsActive()) { opMode.idle(); }
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





    public void end() {
        driveThread.interrupt();
        systemThread.interrupt();
        swerve.disable();

        opMode.stop();
    }




    private void updateDistance() {
        maxDistance = target.hypot(POSE);
        currentDistance = maxDistance;
    }

    private void updateLineEquation() {
        m = (target.y - POSE.y) / (target.x - POSE.x);
        n = target.y - m * target.x;

        a = 1 + m * m;
    }

    private void updateLookupPoint() {
        if (currentDistance < radius) {followPoint = target.point(); return; }

        b = 2 * m * (n - POSE.y) - 2 * POSE.x;
        c = POSE.x * POSE.x + (n - POSE.y) * (n - POSE.y) - radius * radius;

        double delta = b * b - 4 * a * c;
        if (delta < 0) { followPoint = target.point(); return; }

        double x1 = (-b + Math.sqrt(delta)) / (2 * a);
        double x2 = (-b - Math.sqrt(delta)) / (2 * a);

        Point p1 = new Point(x1, m * x1 + n);
        Point p2 = new Point(x2, m * x2 + n);

        followPoint = target.distanceTo(p1) < target.distanceTo(p2) ? p1 : p2;
    }

    private void updateDriveVector() {
        currentDistance = target.hypot(POSE);
        updateLookupPoint();

        driveVector = new Pose(linearCx.calculate(POSE.x, followPoint.x),
                                linearCy.calculate(POSE.y, followPoint.y),
                                angularC.calculate(FindShortestPath(POSE.heading, target.heading))).multiplyBy(13.0 / hardware.batteryVoltage);

        angularC.setP(0.45 + 0.6 * (1 - Range.clip(Math.abs(driveVector.hypot()), 0, 1)));
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