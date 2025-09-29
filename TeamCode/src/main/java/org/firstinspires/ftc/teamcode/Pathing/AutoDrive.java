package org.firstinspires.ftc.teamcode.Pathing;

import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.AngularD;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.AngularP;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.LinearD;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.LinearP;
import static org.firstinspires.ftc.teamcode.Pathing.Math.MathFormulas.FindShortestPath;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Constants.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Constants.Interfaces.Localizer;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Drivetrain.Mecanum.MecanumDrive;
import org.firstinspires.ftc.teamcode.Pathing.Localizer.RR.TwoWheelNew;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Machine;
import org.firstinspires.ftc.teamcode.Pathing.Math.Point;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

import java.util.concurrent.TimeUnit;
import java.util.function.Supplier;

public class AutoDrive {
    private final MecanumDrive drive;


    private Thread driveThread;
    private final LinearOpMode opMode;
    private final ElapsedTime waitTimer;

    private final Hardware hardware;
    private final Localizer localizer;

    public final PIDController linearC, angularC;
    private double busyThreshold = 0.19;

    private double failSafeTimeMs = Double.POSITIVE_INFINITY;
    private final ElapsedTime failSafeTimer = new ElapsedTime();

    private Pose position = new Pose();
    private Pose target = new Pose();
    private Pose driveVector = new Pose();


    private boolean isPaused = false;
    private boolean previousIsPaused = false;
    private boolean usingFailSafe = false;


























    public AutoDrive(LinearOpMode opMode, Machine robot, Pose startPose) {
        this.drive = robot.drive;
        this.position = startPose;

        linearC = new PIDController(LinearP, 0, LinearD);
        angularC = new PIDController(AngularP, 0, AngularD);

        localizer = new TwoWheelNew(opMode);
        setPose(position);

        this.opMode = opMode;
        hardware = Hardware.getInstance(opMode);

        waitTimer = new ElapsedTime();
        startDriveThread();
    }

    public AutoDrive(LinearOpMode opMode, Machine robot) {
        this.drive = robot.drive;

        linearC = new PIDController(LinearP, 0, LinearD);
        angularC = new PIDController(AngularP, 0, AngularD);

        localizer = new TwoWheelNew(opMode);
        setPose(position);

        this.opMode = opMode;
        hardware = Hardware.getInstance(opMode);

        waitTimer = new ElapsedTime();
        startDriveThread();
    }



    private void startDriveThread() {
        driveThread = new Thread(() -> {
                while (opMode.opModeIsActive()) {
                    localizer.update();
                    position = localizer.getRobotPosition();

                    hardware.telemetry.addData("x: ", position.x);
                    hardware.telemetry.addData("y: ", position.y);
                    hardware.telemetry.addData("head: ", position.heading);
                    hardware.telemetry.addData("left: ", drive.left.getFilteredDistance(DistanceUnit.INCH));
                    hardware.telemetry.addData("right: ", drive.right.getFilteredDistance(DistanceUnit.INCH));
                    hardware.telemetry.addData("front: ", drive.front.getFilteredDistance(DistanceUnit.INCH));

                    hardware.telemetry.update();
                    hardware.bulk.clearCache(Enums.Hubs.ALL);

                    if (isPaused && !previousIsPaused) {    // stop the robot when paused
                        drive.update(new Pose());
                        previousIsPaused = true;
                        continue;
                    }

                    if (isPaused) continue;

                    updateDriveVector();

                    if (usingFailSafe && isBusy() && failSafeTimer.time(TimeUnit.MILLISECONDS) > failSafeTimeMs)
                        driveTo(position);

                    drive.update(driveVector);
                }
        });

        driveThread.start();
    }




    public AutoDrive pause() {
        isPaused = true;
        return this;
    }

    public AutoDrive resume() {
        isPaused = false;
        previousIsPaused = false;
        return this;
    }




    public AutoDrive driveTo(Pose pose) {
        this.failSafeTimeMs = Double.POSITIVE_INFINITY;
        this.usingFailSafe = false;
        this.target = new Pose(pose.x, pose.y, normalizeAngleRad(pose.heading));
        return this;
    }

    public AutoDrive driveTo(Pose pose, double ms) {
        this.failSafeTimeMs = ms;
        this.usingFailSafe = true;
        this.target = new Pose(pose.x, pose.y, normalizeAngleRad(pose.heading));

        failSafeTimer.reset();

        return this;
    }








    public AutoDrive waitDrive() { return waitDrive(opMode::idle); }

    public AutoDrive waitDrive(Runnable inLoop) {
        updateDriveVector();
        while (isBusy() && opMode.opModeIsActive()) { inLoop.run(); }

        driveTo(new Pose(position.x, position.y, Math.toRadians(position.heading)));

        return this;
    }

    public AutoDrive waitMs(double ms) {
        waitTimer.reset();
        while (waitTimer.time(TimeUnit.MILLISECONDS) <= ms && opMode.opModeIsActive()) { opMode.idle(); }
        return this;
    }

    public AutoDrive waitAction(Supplier<Boolean> action) {
        while (!action.get() && opMode.opModeIsActive()) { opMode.idle(); }
        return this;
    }

    public AutoDrive waitAction(Supplier<Boolean> action, Runnable inLoop) {
        while (!action.get() && opMode.opModeIsActive()) { inLoop.run(); }
        return this;
    }

    public AutoDrive waitActionTimeFailSafe(Supplier<Boolean> action,
                                            Runnable inLoop,
                                            double ms,
                                            Runnable failSafe) {
        boolean useFailSafe = true;
        waitTimer.reset();

        do {
            inLoop.run();
            if (action.get()) useFailSafe = false;
        } while(!action.get() && opMode.opModeIsActive() && waitTimer.time(TimeUnit.MILLISECONDS) < ms);

        if (useFailSafe)
            failSafe.run();

        return this;
    }

    public AutoDrive waitActionTimeFailSafe(Supplier<Boolean> action,
                                            double ms,
                                            Runnable failSafe) {
         return waitActionTimeFailSafe(action,
                opMode::idle,
                ms,
                failSafe);
    }

    public AutoDrive waitActionTimeFailSafe(Supplier<Boolean> action,
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
        localizer.setPositionEstimate(
                new Pose(pose.x, pose.y, Math.toDegrees(pose.heading))
        );
        return this;
    }





    public AutoDrive disableWheelMotors() {
        drive.disable();
        return this;
    }

    public void end() {
        driveThread.interrupt();
        drive.disable();

        opMode.stop();
    }




    private void updateDriveVector() {
        double radians = normalizeAngleRad(Math.toRadians(position.heading));
        double targetVelX, targetVelY;


        if (Math.abs(target.x - position.x) > 4.6) {
            linearC.setP(LinearP);
            targetVelX = linearC.calculate(position.x, target.x);
        } else {
            linearC.setP(0.04);
            targetVelX = linearC.calculate(position.x, target.x);
        }


        if (Math.abs(target.y - position.y) > 4) {
            linearC.setP(LinearP);
            targetVelY = linearC.calculate(position.y, target.y);
        } else {
            linearC.setP(0.04);
            targetVelY = linearC.calculate(position.y, target.y);
        }

        double targetVelHeading = angularC.calculate(FindShortestPath(radians, target.heading));

        double velocityMagnitude = Math.sqrt(targetVelX * targetVelX + targetVelY * targetVelY);
        double turnRadius = velocityMagnitude / (Math.abs(targetVelHeading) + 1e-6);
        double centripetalAcc = velocityMagnitude * velocityMagnitude / (turnRadius + 1e-6);

        double driftCompensationX = -centripetalAcc * Math.sin(radians);
        double driftCompensationY = centripetalAcc * Math.cos(radians);

        targetVelX += driftCompensationX - 0.1;
        targetVelY += driftCompensationY;

        // convert the adjusted vector to the field-centric coordinate system.
        Pose vector = new Pose(targetVelX, targetVelY, 0);
        Point rotatedDiff = vector.point().rotate_matrix(-radians);

        // build the final drive vector using the rotated linear outputs and the angular command.
        driveVector = new Pose(
                rotatedDiff.x,
                rotatedDiff.y,
                targetVelHeading).multiplyBy(13.0 / hardware.batteryVoltageSensor.getVoltage());
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


    public Pose getPosition() { return position; }

    public boolean isBusy() { return !driveVector.closeToZero(busyThreshold); }




}
