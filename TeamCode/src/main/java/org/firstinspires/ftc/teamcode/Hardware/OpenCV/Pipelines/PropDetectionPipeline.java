package org.firstinspires.ftc.teamcode.Hardware.OpenCV.Pipelines;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.autoOnBlue;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class PropDetectionPipeline extends OpenCvPipeline {

    private Scalar lowerBlue = new Scalar(90, 100, 100);
    private Scalar upperBlue = new Scalar(130, 255, 255);

    private Scalar lowerRed1 = new Scalar(0, 100, 20);
    private Scalar upperRed1 = new Scalar(10, 255, 255);

    private Scalar lowerRed2 = new Scalar(160, 100, 20);
    private Scalar upperRed2 = new Scalar(179, 255, 255);

    Point center = new Point();

    @Override
    public Mat processFrame(Mat input) {
        Mat inputToHSV = new Mat();
        Mat detections = new Mat();

        Imgproc.cvtColor(input, inputToHSV, Imgproc.COLOR_RGB2HSV);

        if (autoOnBlue)
            Core.inRange(inputToHSV, lowerBlue, upperBlue, detections);
        else {
            Core.inRange(inputToHSV, lowerRed1, upperRed1, detections);
            Core.inRange(inputToHSV, lowerRed2, upperRed2, detections);
        }

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        Imgproc.findContours(detections, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);


        MatOfPoint largestContour = contours.stream()
                .max(Comparator.comparingDouble(Imgproc::contourArea))
                .orElse(null);



        if (largestContour != null) {
            center = new Point();
            Moments moments = Imgproc.moments(largestContour);
            center.x = moments.m10 / moments.m00;
            center.y = moments.m01 / moments.m00;

            Imgproc.circle(input, center, 10, new Scalar(128, 0, 128), -1);  // Draw the center on the frame
            Imgproc.drawContours(input, Collections.singletonList(largestContour), -1, new Scalar(0, 255, 0), 3);
        } else { center = null; }


        inputToHSV.release();
        detections.release();
        hierarchy.release();
        if (largestContour != null) largestContour.release();

        if (contours != null) for (MatOfPoint mat : contours) { mat.release(); }

        return input;
    }


    public Point getCenter() { return center; }
}
