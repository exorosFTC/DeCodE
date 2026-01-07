package org.firstinspires.ftc.teamcode.Pathing;

import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoAngularD;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoAngularP;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoLinearD;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.AutoLinearP;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.POSE;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.startPose;
import static org.firstinspires.ftc.teamcode.Pathing.Math.MathFormulas.FindShortestPath;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandBase.Constants.Enums;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.ScoringSystem;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Pathing.Math.Point;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

import java.util.concurrent.TimeUnit;
import java.util.function.BooleanSupplier;

public class AutoDrive {
    private final Hardware hardware;
    private final SwerveDrive swerve;
    private final ScoringSystem system;


    private Thread driveThread, systemThread;
    private final LinearOpMode opMode;
    private final ElapsedTime waitTimer;

    public final PIDController linearC, angularC;
    private double busyThreshold = 0.1;

    private double failSafeTimeMs = Double.POSITIVE_INFINITY;
    private final ElapsedTime failSafeTimer = new ElapsedTime();

    private Pose target = new Pose();
    private Pose driveVector = new Pose();
    public PurePursuitController controller = null;

    private boolean isPaused = false;
    private boolean previousIsPaused = false;
    private boolean usingFailSafe = false;




    public AutoDrive(LinearOpMode opMode, SwerveDrive swerve, ScoringSystem system, Pose startPose) {
        this.swerve = swerve;
        this.system = system;
        POSE = startPose;

        hardware = Hardware.getInstance(opMode);
        setPose(POSE);

        linearC = new PIDController(AutoLinearP, 0, AutoLinearD);
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

                    if (isPaused && !previousIsPaused) {    // stop the robot when paused
                        swerve.setLockedX(true);
                        previousIsPaused = true;
                        continue;
                    }

                    if (isPaused) continue;

                    if (controller != null) {
                        target = controller.update();
                    }
                    updateDriveVector();

                    hardware.telemetry.addData("x", POSE.x);
                    hardware.telemetry.addData("y", POSE.y);
                    hardware.telemetry.addData("head", Math.toDegrees(POSE.heading));
                    hardware.telemetry.addData("target x", target.x);
                    hardware.telemetry.addData("target y", target.y);
                    hardware.telemetry.addData("target head", Math.toDegrees(target.heading));
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
            opMode.waitForStart();

            while (opMode.opModeIsActive()) {
                hardware.bulk.clearCache(Enums.Hubs.ALL);
                hardware.read(system);

                system.shooter.update();
                system.write();

                Thread.yield();
            }
        });

        driveThread.start();
        systemThread.start();
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




    @Deprecated
    public AutoDrive driveTo(Pose pose) {
        this.failSafeTimeMs = Double.POSITIVE_INFINITY;
        this.usingFailSafe = false;
        this.target = new Pose(pose.x, pose.y, normalizeAngleRad(pose.heading));
        return this;
    }

    @Deprecated
    public AutoDrive driveTo(Pose pose, double ms) {
        this.failSafeTimeMs = ms;
        this.usingFailSafe = true;
        this.target = new Pose(pose.x, pose.y, normalizeAngleRad(pose.heading));

        failSafeTimer.reset();

        return this;
    }

    public AutoDrive drivePath(PurePursuitController controller) {
        if (this.controller == null || this.controller.isBusy()) this.controller = controller;
        return this;
    }








    public AutoDrive waitDrive() { return waitDrive(opMode::idle); }

    public AutoDrive waitDrive(Runnable inLoop) {
        updateDriveVector();
        while (isBusy() && opMode.opModeIsActive()) { inLoop.run(); }

        driveTo(new Pose(POSE.x, POSE.y, POSE.heading));

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

    public AutoDrive setBusyThreshold(double threshold) {
        this.busyThreshold = threshold;
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
        swerve.disable();

        opMode.stop();
    }




    private void updateDriveVector() {
        double targetVelX, targetVelY, targetVelHeading;

        targetVelX = swerve.xLim.calculate(linearC.calculate(POSE.x, target.x));
        targetVelY = swerve.yLim.calculate(linearC.calculate(POSE.y, target.y));
        targetVelHeading = angularC.calculate(FindShortestPath(POSE.heading, target.heading));

        //if (new Point(targetVelX, targetVelY).closeToZero(busyThreshold * 2)) angularC.setP(0.4);
        //else angularC.setP(0.1);

        driveVector = new Pose(
                Math.abs(targetVelX) > busyThreshold ? targetVelX : 0,
                Math.abs(targetVelY) > busyThreshold ? targetVelY : 0,
                Math.abs(targetVelHeading) > busyThreshold ? targetVelHeading : 0
                ).multiplyBy(13.5 / hardware.batteryVoltage);
    }





    public void setLinearPID(double p, double i, double d) { linearC.setPID(p, i, d); }

    public void setAngularPID(double p, double i, double d) { angularC.setPID(p, i, d); }




    private double normalizeAngleRad(double angle) {
        if (angle < 0) return angle + 2 * Math.PI;
        return angle;
    }

    private double normalizeAngleDeg(double angle) {
        if (angle < 0) return angle + 360;
        return angle;
    }


    public Pose getPosition() { return POSE; }

    public boolean isBusy() { return !driveVector.closeToZero(busyThreshold); }




}
