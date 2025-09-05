package org.firstinspires.ftc.teamcode.Hardware.OpenCV;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.infinity;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class AprilTagCamera extends Camera {
   private LinearOpMode opMode;
   private Telemetry telemetry;

   private AprilTagProcessor aprilTag;
   private VisionPortal vision;

   private final int WIDTH = 640, LENGTH = 360;
   private final Size resolution = new Size(WIDTH, LENGTH);

    public AprilTagCamera(LinearOpMode opMode, Telemetry telemetry) {
        super(opMode, telemetry);

        this.opMode = opMode;
        this.telemetry = telemetry;

        initCamera();
    }

    private void initCamera() {
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .build();

        vision = new VisionPortal.Builder()
                .setCamera(opMode.hardwareMap.get(WebcamName.class, super.getCameraName()))
                .setCameraResolution(resolution)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .addProcessor(aprilTag)
                .build();
    }

    public double getDetection() {
        List<AprilTagDetection> detections = new ArrayList<>();
        double smallestDistanceToDetection = infinity;

        if (aprilTag.getDetections().size() != 0)
            detections = aprilTag.getDetections();

        for (AprilTagDetection detection : detections) {
            double currentDistance = detection.ftcPose.y;
            smallestDistanceToDetection = Math.min(smallestDistanceToDetection, currentDistance);
        }

        return smallestDistanceToDetection;
    }


}
