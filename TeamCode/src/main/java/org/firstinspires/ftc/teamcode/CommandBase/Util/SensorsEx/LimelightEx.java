package org.firstinspires.ftc.teamcode.CommandBase.Util.SensorsEx;

import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.POSE;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.SystemConstants.autoOnBlue;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.SystemConstants.lastValidRandomization;
import static org.firstinspires.ftc.teamcode.CustomPathing.Math.MathFormulas.normalizeRadians;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.CustomPathing.Localizer.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.CustomPathing.Math.Geometry.Pose;

public class LimelightEx {
    public enum Pipeline{
        RANDOMIZATION,
        BLUE_GOAL,
        RED_GOAL
    }

    public enum Randomization{
        LEFT,
        CENTER,
        RIGHT
    }



    private final Limelight3A limelight;
    private final LinearOpMode opMode;

    public boolean enabled = false;

    public static double KALMAN_GAIN_POS = 0.1;
    public static double KALMAN_FILTER_INTERVAL_S = 2;


    public LimelightEx.Pipeline pipeline = LimelightEx.Pipeline.RANDOMIZATION;
    public LLResult result = null;

    public LimelightEx(String name, LinearOpMode opMode) {
        limelight = opMode.hardwareMap.get(Limelight3A.class, name);
        this.opMode = opMode;
    }


    public void start() { enabled = true; limelight.start(); }

    public void stop() { enabled = false; limelight.stop(); result = null; }

    public void read() {
        if (!enabled)  return;

        result = limelight.getLatestResult();
    }



    public void setPipeline(LimelightEx.Pipeline pipeline) {
        this.pipeline = pipeline;

        switch (pipeline) {
            case RANDOMIZATION:
                limelight.pipelineSwitch(0);
                break;
            case BLUE_GOAL:
                limelight.pipelineSwitch(1);
                break;
            case RED_GOAL:
                limelight.pipelineSwitch(2);
                break;
        }
    }



    public LimelightEx.Randomization getRandomization() {
        if (pipeline != LimelightEx.Pipeline.RANDOMIZATION) setPipeline(LimelightEx.Pipeline.RANDOMIZATION);
        if (result == null || !result.isValid()) return lastValidRandomization;
        if (result.getFiducialResults().isEmpty()) return lastValidRandomization;

        int tagId = result.getFiducialResults().get(0).getFiducialId();
        switch (tagId) {
            case 21: { lastValidRandomization = LimelightEx.Randomization.LEFT; } break;
            case 22: { lastValidRandomization = LimelightEx.Randomization.CENTER; } break;
            case 23: { lastValidRandomization = LimelightEx.Randomization.RIGHT; } break;
            default: {} break;
        }

        return lastValidRandomization;
    }

    public LimelightEx.Pipeline getPipeline() { return pipeline; }

    public double getCenterOffset() {
        if (autoOnBlue && pipeline != LimelightEx.Pipeline.BLUE_GOAL) setPipeline(LimelightEx.Pipeline.BLUE_GOAL);
        else if (!autoOnBlue && pipeline != LimelightEx.Pipeline.RED_GOAL) setPipeline(LimelightEx.Pipeline.RED_GOAL);
        if (result == null) return 0;

        return -result.getTx();
    }



    public void relocalize(PinpointLocalizer localizer) {
        if (!enabled) start();
        setPipeline(autoOnBlue ? LimelightEx.Pipeline.BLUE_GOAL : LimelightEx.Pipeline.RED_GOAL);

        new Thread(() -> {
            ElapsedTime timer = new ElapsedTime();

            while (timer.seconds() <= KALMAN_FILTER_INTERVAL_S) {
                read();

                if (result == null) continue;
                Pose3D botPose = result.getBotpose();
                if (botPose == null) continue;

                double filteredX = kalmanUpdate(POSE.x, botPose.getPosition().x * 100, KALMAN_GAIN_POS);
                double filteredY = kalmanUpdate(POSE.x, botPose.getPosition().y * 100, KALMAN_GAIN_POS);
                localizer.setPositionEstimate(new Pose(filteredX, filteredY, POSE.heading));
            }
        }).start();
    }



    public boolean tagInSight() {
        return !result.getFiducialResults().isEmpty() && pipeline != LimelightEx.Pipeline.RANDOMIZATION;
    }




    private static double kalmanUpdate(double state, double measurement, double gain) {
        return state + (gain * (measurement - state));
    }
}
