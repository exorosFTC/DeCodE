package org.firstinspires.ftc.teamcode.CustomPathing;

import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoAngularD;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoAngularP;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoAngularVelocityMultiplier;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoLinearDx;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoLinearDy;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoLinearPx;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoLinearPy;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.POSE;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.startPose;
import static org.firstinspires.ftc.teamcode.CustomPathing.Math.MathFormulas.FindShortestPath;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.ScoringSystem;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Hardware;
import org.firstinspires.ftc.teamcode.CommandBase.Util.SensorsEx.HubBulkRead;
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

    private double busyThresholdLinear = 0.96,
                   busyThresholdAngular = Math.toRadians(6);

    private double failSafeTimeMs = Double.POSITIVE_INFINITY;
    private final ElapsedTime failSafeTimer = new ElapsedTime();

    private Pose target = new Pose();
    private Pose driveVector = new Pose();

    private boolean usingFailSafe = false;
    private double maxDistance = 0;
    private double currentDistance = 0;





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
                swerve.write();

                hardware.telemetry.addData("x", POSE.x);
                hardware.telemetry.addData("y", POSE.y);
                hardware.telemetry.addData("head", Math.toDegrees(POSE.heading));
                hardware.telemetry.addData("target x", target.x);
                hardware.telemetry.addData("target y", target.y);
                hardware.telemetry.addData("target head", Math.toDegrees(target.heading));
                hardware.telemetry.update();

                if (usingFailSafe && isBusy() && failSafeTimer.time(TimeUnit.MILLISECONDS) > failSafeTimeMs)
                    driveTo(POSE);

                updateDriveVector();
                swerve.update(driveVector);
            }

            startPose = POSE;
        });
        systemThread = new Thread(() -> {
            opMode.waitForStart();

            while (opMode.opModeIsActive()) {
                hardware.bulk.clearCache(HubBulkRead.Hubs.ALL);
                hardware.read(system);

                system.shooter.update();
                system.write();

                Thread.yield();
            }
        });

        driveThread.start();
        systemThread.start();
    }




    public AutoDrive driveTo(Pose pose) {
        driveTo(pose, Double.POSITIVE_INFINITY);
        return this;
    }

    public AutoDrive driveTo(Pose pose, double ms) {
        this.failSafeTimeMs = ms;
        this.usingFailSafe = ms != Double.POSITIVE_INFINITY;

        this.target = new Pose(pose.x, pose.y, normalizeAngleRad(pose.heading));
        maxDistance = target.hypot(POSE);
        currentDistance = maxDistance;

        failSafeTimer.reset();

        return this;
    }








    public AutoDrive waitDrive() { return waitDrive(opMode::idle, 0.96); }

    public AutoDrive waitDrive(double threshold) { return waitDrive(opMode::idle, threshold); }

    public AutoDrive waitDrive(Runnable inLoop) { return waitDrive(inLoop, 0.96); }

    public AutoDrive waitDrive(Runnable inLoop, double threshold) {
        this.busyThresholdLinear = threshold;

        updateDriveVector();
        try { Thread.sleep(10); } catch (InterruptedException e) {}

        while (isBusy() && opMode.opModeIsActive()) { inLoop.run(); }
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

    public AutoDrive waitActionTimeFailSafe(BooleanSupplier action,
                                            Runnable inLoop,
                                            double ms,
                                            Runnable failSafe) {
        boolean useFailSafe = true;
        waitTimer.reset();

        do {
            inLoop.run();
            if (action.getAsBoolean()) useFailSafe = false;
        } while(!action.getAsBoolean() && opMode.opModeIsActive() && waitTimer.time(TimeUnit.MILLISECONDS) < ms);

        if (useFailSafe)
            failSafe.run();

        return this;
    }

    public AutoDrive waitActionTimeFailSafe(BooleanSupplier action,
                                            double ms,
                                            Runnable failSafe) {
        return waitActionTimeFailSafe(action,
                opMode::idle,
                ms,
                failSafe);
    }

    public AutoDrive waitActionTimeFailSafe(BooleanSupplier action,
                                            double ms) {
        return waitActionTimeFailSafe(action,
                opMode::idle,
                ms,
                opMode::idle);
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





    public AutoDrive disableDrive() {
        swerve.disable();
        return this;
    }

    public void end() {
        driveThread.interrupt();
        systemThread.interrupt();
        swerve.disable();

        opMode.stop();
    }




    private void updateDriveVector() {
        double targetVelX, targetVelY, targetVelHeading;
        currentDistance = target.hypot(POSE);

        targetVelX = linearCx.calculate(POSE.x, target.x);
        targetVelY = linearCy.calculate(POSE.y, target.y);
        targetVelHeading = angularC.calculate(FindShortestPath(POSE.heading, target.heading));

        driveVector = new Pose(targetVelX, targetVelY, targetVelHeading).multiplyBy(13.0 / hardware.batteryVoltage);
    }


    private double normalizeAngleRad(double angle) {
        if (angle < 0) return angle + 2 * Math.PI;
        return angle;
    }

    public boolean isBusy() {
        return currentDistance > maxDistance * (1 - busyThresholdLinear)
                                            &&
               Math.abs(target.heading - POSE.heading) > busyThresholdAngular;
    }




}