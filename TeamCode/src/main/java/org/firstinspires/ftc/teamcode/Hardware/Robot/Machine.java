package org.firstinspires.ftc.teamcode.Hardware.Robot;


import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants.driveSensitivity;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.randomization;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.telemetryAddLoopTime;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.usingAprilTagCamera;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.usingOpenCvCamera;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.OpenCV.AprilTagCamera;
import org.firstinspires.ftc.teamcode.Hardware.OpenCV.Camera;
import org.firstinspires.ftc.teamcode.Hardware.OpenCV.Pipelines.PropDetectionPipeline;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Systems.Drivetrain;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

import javax.annotation.Nullable;

public class Machine {
    public Hardware hardware;
    public Thread drivetrainThread;

    public Drivetrain drive;

    public GamepadEx g1, g2;
    private boolean add_g1, add_g2;

    private double startLoopTime, endLoopTime;
    private final int secondsToNanoseconds = 1000000000;

    private Camera openCvCamera;
    private AprilTagCamera aprilTagCamera;

    private LinearOpMode opMode;


    //default data
    private MachineData data = new MachineData()
            .add(Enums.OpMode.TELE_OP)
            .setUsingAprilTag(false)
            .setUsingOpenCv(false);




    public Machine addData(MachineData data) {
        this.data = data;
        return this;
    }

    public Machine addGamepads(Enums.Gamepads gamepads) {
        switch (gamepads) {
            case G1: {
                add_g1 = true;
                add_g2 = false;
            } break;

            case G2: {
                add_g1 = false;
                add_g2 = true;
            } break;

            case BOTH: {
                add_g1 = true;
                add_g2 = true;
            } break;
        }

        return this;
    }

    public Machine construct(LinearOpMode opMode) {
        this.hardware = Hardware.getInstance(opMode);
        this.opMode = opMode;

        //follower = new Follower(hardware);
        drive = new Drivetrain(opMode);

        g1 = add_g1 ? new GamepadEx(opMode.gamepad1) : null;
        g2 = add_g2 ? new GamepadEx(opMode.gamepad2) : null;

        openCvCamera = usingOpenCvCamera ? new Camera(opMode, hardware.telemetry) : null;
        aprilTagCamera = usingAprilTagCamera ? new AprilTagCamera(opMode, hardware.telemetry) : null;

        if (SystemConstants.opModeType != Enums.OpMode.AUTONOMUS)
            createG1Thread();

        return this;
    }





    private void createG1Thread() {
        if (g1 != null)
            drivetrainThread = new Thread(() -> {
                while (opMode.opModeIsActive()) {
                    drive.update(new Pose(
                            g1.getLeftY(),                         // forwards
                            -g1.getLeftX(),                        // sideways
                            -g1.getRightX() * driveSensitivity      // turning
                            ));

                    g1.readButtons();
                }
            });

        else drivetrainThread = new Thread(() -> {
            while (opMode.opModeIsActive()) {
                drive.update(new Pose(
                        g2.getLeftY(),                         // forwards
                        -g2.getLeftX(),                        // sideways
                        -g2.getRightX() * driveSensitivity      // turning
                ));

                //g2.readButtons();
            }
        });
    }







    private void getStartingLoopTime() { startLoopTime = endLoopTime; }

    private void getEndingLoopTime() { endLoopTime = System.nanoTime(); }

    public double getLoopFrequency() { return telemetryAddLoopTime ? secondsToNanoseconds / (endLoopTime - startLoopTime) : 0; }




    public double getClosestAprilTag() { return (aprilTagCamera == null) ? 0 : aprilTagCamera.getDetection(); }

    public void searchForProp() {
        if (openCvCamera == null || !(openCvCamera.getPipeline() instanceof PropDetectionPipeline)) { randomization = null; }
            else {


                Enums.Randomization detection = openCvCamera.getRandomization();


                if (detection != null)
                    randomization = detection;
        }
    }

    @Nullable
    public Enums.Randomization getPropPosition() { return (usingOpenCvCamera) ? randomization : null; }

    public void setPipeline(Enums.Pipelines pipeline) { if (usingOpenCvCamera) openCvCamera.setPipeline(pipeline); }

    public void closeCamera() { if (usingOpenCvCamera) openCvCamera.close(); }




    public void initComplete() {
        hardware.telemetry.addLine("INIT COMPLETE! KILL EM' ALL 😈");
        hardware.telemetry.update();
    }

    public void addTelemetry(String caption, Object value) { hardware.telemetry.addData(caption, value); }

    public void clearTelemetry() { hardware.telemetry.clearAll(); }

    public void updateTelemetry() {
        if (telemetryAddLoopTime) {
            getEndingLoopTime();
            hardware.telemetry.addData("Loop Time: ", getLoopFrequency());
            getStartingLoopTime();
            }
        hardware.telemetry.update();
    }

    public void debuggingTelemetry() {}

    public void setAccess(Enums.Access access) {
        switch (access) {
            case INTAKE: {
                opMode.gamepad2.rumble(0, 1, 500);
                opMode.gamepad2.setLedColor(0.96, 0.54,  0.09, 120000);
            } break;

            case OUTTAKE: {
                opMode.gamepad2.rumble(1, 0, 500);
                opMode.gamepad2.setLedColor(0.5, 0,  0.5, 120000);
            } break;
        }
    }




}
