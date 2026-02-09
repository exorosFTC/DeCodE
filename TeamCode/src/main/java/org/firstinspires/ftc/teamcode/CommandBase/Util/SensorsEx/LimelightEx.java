package org.firstinspires.ftc.teamcode.CommandBase.Util.SensorsEx;

import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.POSE;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.SystemConstants.autoOnBlue;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.SystemConstants.lastValidRandomization;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
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
        if (result == null) return lastValidRandomization;
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



    public Pose relocalize() {
        if (!enabled) start();
        setPipeline(autoOnBlue ? LimelightEx.Pipeline.BLUE_GOAL : LimelightEx.Pipeline.RED_GOAL);

        final int SAMPLES = 10;
        int count = 0;

        double sumX = 0;
        double sumY = 0;

        while (opMode.opModeIsActive() && count < SAMPLES) {
            read();

            if (result == null) continue;
            if (!result.isValid()) continue;

            Pose3D llPose = result.getBotpose();
            if (llPose == null) continue;

            sumX += -100 * llPose.getPosition().x;
            sumY += -100 * llPose.getPosition().y;
            count++;
        }

        if (count == 0) return POSE;

        return new Pose(
                sumX / count,
                sumY / count,
                POSE.heading
        );
    }



    public boolean tagInSight() {
        return !result.getFiducialResults().isEmpty() && pipeline != LimelightEx.Pipeline.RANDOMIZATION;
    }

}
