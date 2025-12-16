package org.firstinspires.ftc.teamcode.CommandBase.Util.SensorsEx;

import static org.firstinspires.ftc.teamcode.CommandBase.Constants.SystemConstants.autoOnBlue;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CommandBase.Constants.Enums;

public class LimelightEx {
    private final Limelight3A limelight;

    public boolean enabled = false;

    public Enums.Pipeline pipeline = Enums.Pipeline.RANDOMIZATION;
    public static Enums.Randomization lastValidRandomization = Enums.Randomization.LEFT;
    public LLResult
            raw = null,
            result = null;

    public LimelightEx(String name, HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, name);
    }


    public void start() { enabled = true; limelight.start(); }

    public void stop() { enabled = false; limelight.stop(); }

    public void read() {
        if (!enabled)  return;

        raw = limelight.getLatestResult();
        result = (raw == null || !result.isValid()) ? result : raw;
    }



    public void setPipeline(Enums.Pipeline pipeline) {
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



    public Enums.Randomization getRandomization() {
        if (pipeline != Enums.Pipeline.RANDOMIZATION) setPipeline(Enums.Pipeline.RANDOMIZATION);
        if (result == null) return lastValidRandomization;

        int tagId = result.getFiducialResults().get(0).getFiducialId();
        switch (tagId) {
            case 21: { lastValidRandomization = Enums.Randomization.LEFT; } break;
            case 22: { lastValidRandomization = Enums.Randomization.CENTER; } break;
            case 23: { lastValidRandomization = Enums.Randomization.RIGHT; } break;
            default: {} break;
        }
        return lastValidRandomization;
    }

    public Enums.Pipeline getPipeline() { return pipeline; }

    public double getCenterOffset() {
        if (autoOnBlue && pipeline != Enums.Pipeline.BLUE_GOAL) setPipeline(Enums.Pipeline.BLUE_GOAL);
        else if (!autoOnBlue && pipeline != Enums.Pipeline.RED_GOAL) setPipeline(Enums.Pipeline.RED_GOAL);
        if (result == null) return 0;

        return result.getTx();
    }

}
