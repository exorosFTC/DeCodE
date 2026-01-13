package org.firstinspires.ftc.teamcode.CustomPathing;

import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoAngularD;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoAngularP;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoLinearDx;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoLinearDy;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoLinearPx;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoLinearPy;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoVelocityMultiplier;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.POSE;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.goalPosition;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.goalPositionBlue;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.goalPositionRed;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.startPose;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.SystemConstants.autoOnBlue;
import static org.firstinspires.ftc.teamcode.CustomPathing.Math.MathFormulas.normalizeRadians;
import static org.firstinspires.ftc.teamcode.CustomPathing.Math.MathFormulas.normalizeRadiansPositive;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandBase.Constants.Enums;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.ScoringSystem;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Hardware;
import org.firstinspires.ftc.teamcode.CustomPathing.Math.Geometry.Point;
import org.firstinspires.ftc.teamcode.CustomPathing.Math.Geometry.Pose;

import java.util.concurrent.TimeUnit;
import java.util.function.BooleanSupplier;

public class AutoDrive {
    private final Hardware hardware;
    private final SwerveDrive swerve;
    private final ScoringSystem system;

    private Thread swerveThread, systemThread;
    private final LinearOpMode opMode;

    private final ElapsedTime waitTimer;
    public final PIDController linearCx, linearCy, angularC;

    private double busyThresholdLinear = 0.08,
                    busyThresholdAngular = Math.toRadians(7);
    private double thresholdMultiplier = 1;

    private double failSafeTimeMs = Double.POSITIVE_INFINITY;
    private final ElapsedTime failSafeTimer = new ElapsedTime();

    private Pose target = new Pose();
    private Pose driveVector = new Pose();

    private boolean isPaused = false;
    private boolean previousIsPaused = false;
    private boolean usingFailSafe = false;




    public AutoDrive(LinearOpMode opMode, SwerveDrive swerve, ScoringSystem system, Pose startPose) {
        this.swerve = swerve;
        this.system = system;

        hardware = Hardware.getInstance(opMode);
        setPose(startPose);

        linearCx = new PIDController(AutoLinearPx, 0, AutoLinearDx);
        linearCy = new PIDController(AutoLinearPy, 0, AutoLinearDy);
        angularC = new PIDController(AutoAngularP, 0, AutoAngularD);

        this.opMode = opMode;

        waitTimer = new ElapsedTime();
        startThreads();
    }

    private void startThreads() {
        swerveThread = new Thread(() -> {
                opMode.waitForStart();

                while (opMode.opModeIsActive()) {
                    swerve.read();
                    swerve.write();

                    if (isPaused && !previousIsPaused) {    // stop the robot when paused
                        swerve.setLockedX(true);
                        swerve.update(new Pose());

                        previousIsPaused = true;
                        continue;
                    }

                    if (isPaused) continue;
                    updateDriveVector();

                    hardware.telemetry.addData("x", POSE.x);
                    hardware.telemetry.addData("y", POSE.y);
                    hardware.telemetry.addData("head", Math.toDegrees(POSE.heading));
                    //hardware.telemetry.addData("target x", target.x);
                    //hardware.telemetry.addData("target y", target.y);
                    //hardware.telemetry.addData("target head", Math.toDegrees(target.heading));
                    //hardware.telemetry.addData("randomization", SystemConstants.lastValidRandomization);
                    hardware.telemetry.update();

                    if (usingFailSafe && isBusy() && failSafeTimer.time(TimeUnit.MILLISECONDS) > failSafeTimeMs)
                        driveTo(POSE);

                    swerve.update(driveVector);
                }

                // prevent inertia movement from the drivetrain
                swerve.setLockedX(true);
                swerve.update(new Pose());

                startPose = POSE;
        });
        systemThread = new Thread(() -> {
            if (autoOnBlue) goalPosition = goalPositionBlue;
            else goalPosition = goalPositionRed;

            opMode.waitForStart();

            while (opMode.opModeIsActive()) {
                hardware.bulk.clearCache(Enums.Hubs.ALL);
                hardware.read(system);

                system.shooter.update();
                system.write();

                Thread.yield();
            }
        });

        swerveThread.start();
        systemThread.start();
    }

    public void endThreads() {
        swerveThread.interrupt();
        systemThread.interrupt();
    }



    public AutoDrive pause() {
        isPaused = true;
        return this;
    }

    public AutoDrive resume() {
        isPaused = false;
        previousIsPaused = false;
        swerve.setLockedX(false);

        return this;
    }



    public AutoDrive driveTo(Pose pose) {
        this.failSafeTimeMs = Double.POSITIVE_INFINITY;
        this.usingFailSafe = false;
        this.target = new Pose(pose.x, pose.y, normalizeRadiansPositive(pose.heading));
        return this;
    }

    public AutoDrive driveTo(Pose pose, double ms) {
        this.failSafeTimeMs = ms;
        this.usingFailSafe = true;
        this.target = new Pose(pose.x, pose.y, normalizeRadiansPositive(pose.heading));

        failSafeTimer.reset();

        return this;
    }



    public AutoDrive waitDrive() { return waitDrive(opMode::idle, 1); }

    public AutoDrive waitDrive(double thresholdMultiplier) { return waitDrive(opMode::idle, thresholdMultiplier); }

    public AutoDrive waitDrive(Runnable inLoop) { return waitDrive(inLoop, 1); }

    public AutoDrive waitDrive(Runnable inLoop, double thresholdMultiplier) {
        this.thresholdMultiplier = thresholdMultiplier;

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
        endThreads();

        swerve.disable();
        opMode.stop();
    }



    private void updateDriveVector() {
        double xDiff = target.x - POSE.x;
        double yDiff = target.y - POSE.y;
        double headDiff = normalizeRadians(target.heading - POSE.heading);

        // transform it to field centric
        Point linear = new Point(xDiff, yDiff).rotate_matrix(-POSE.heading);

        double power = linear.hypot();
        //angularC.setP(AutoAngularP + power * AutoVelocityMultiplier);

        double xPower = linearCx.calculate(-linear.x, 0);
        double yPower = linearCy.calculate(-linear.y, 0);
        double headPower = angularC.calculate(-headDiff, 0);

        driveVector = new Pose(
                Math.abs(xPower) > busyThresholdLinear ? xPower : 0,
                Math.abs(yPower) > busyThresholdLinear ? yPower : 0,
                headPower
                ).norm().multiplyBy(12.0 / hardware.batteryVoltage);
    }



    public boolean isBusy() { return !(
            new Point(Math.abs(target.x - POSE.x), Math.abs(target.y - POSE.y)).closeToZero(6 * thresholdMultiplier)
                    &&
            driveVector.point().closeToZero(busyThresholdLinear * thresholdMultiplier)
                    &&
            Math.abs(target.heading - POSE.heading) < busyThresholdAngular);
    }




}
