package org.firstinspires.ftc.teamcode.Hardware.OpenCV;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.cameraConfigurationName;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.OpenCV.Pipelines.PropDetectionPipeline;
import org.firstinspires.ftc.teamcode.Hardware.OpenCV.Pipelines.SampleDetectionPipeline;
import org.firstinspires.ftc.teamcode.Hardware.Util.SensorsEx.ColorEx;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import javax.annotation.Nullable;

public class Camera {

    private OpenCvCamera camera;
    private OpenCvCameraRotation cameraOrientation = OpenCvCameraRotation.UPRIGHT;
    private OpenCvPipeline currentPipeline;

    private int HEIGHT = 240, WIDTH = 320;
    private boolean isOpened = false;

    private LinearOpMode opMode;
    private Telemetry dashboardTelemetry;

    public Camera(LinearOpMode opMode, Telemetry dashboardTelemetry) {
        this.dashboardTelemetry = dashboardTelemetry;
        this.opMode = opMode;

        initCamera();
    }

    private void initCamera(){
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName.class, cameraConfigurationName), cameraMonitorViewId);
    }

    public void setPipeline(Enums.Pipelines desiredPipeline) {
        if (isOpened) {
            camera.closeCameraDeviceAsync(() -> {});
            isOpened = false;
        }

        switch (desiredPipeline) {
            case DETECTING_PROP: { currentPipeline = new PropDetectionPipeline(); }
            case DETECTING_SAMPLE: { currentPipeline = new SampleDetectionPipeline(); }
            default: {}
        }

        if (currentPipeline != null)
            camera.setPipeline(currentPipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(WIDTH, HEIGHT, cameraOrientation);
                FtcDashboard.getInstance().startCameraStream(camera, 0);

                isOpened = true;
            }
            @Override
            public void onError(int errorCode) {
                dashboardTelemetry.addLine("ERROR: couldn't open camera");
                dashboardTelemetry.update();
            }
        });
    }

    public OpenCvPipeline getPipeline() { return currentPipeline; }

    public boolean isOpened() { return isOpened; }

    public void setCameraParameters(int width, int height) {
        WIDTH = width;
        HEIGHT = height;
    }

    public void setCameraParameters(OpenCvCameraRotation cameraOrientation) { this.cameraOrientation = cameraOrientation; }

    public void setCameraParameters(int width, int height, OpenCvCameraRotation cameraOrientation) {
        setCameraParameters(width, height);
        setCameraParameters(cameraOrientation);
    }

    public void close() { camera.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
        @Override
        public void onClose() { opMode.idle(); }
    }); }

    protected String getCameraName() { return cameraConfigurationName; }

    public int getWidth() { return WIDTH; }

    public int getHeight() { return HEIGHT; }

    public OpenCvCameraRotation getOrientation() { return cameraOrientation; }




    @Nullable
    public Enums.Randomization getRandomization() {
        if (!(currentPipeline instanceof PropDetectionPipeline))
            return null;

        Point center = ((PropDetectionPipeline) currentPipeline).getCenter();

        if (center != null) {

        if (center.x <= WIDTH / 3) return Enums.Randomization.LEFT;

        if (center.x <= 2 * WIDTH / 3) return Enums.Randomization.CENTER;

        return Enums.Randomization.RIGHT;

        } else return null;
    }




    public org.firstinspires.ftc.teamcode.Pathing.Math.Point getSampleCenterError() {
        if (!(currentPipeline instanceof SampleDetectionPipeline))
            return new org.firstinspires.ftc.teamcode.Pathing.Math.Point();

        Point sampleCenterCameraCoords = ((SampleDetectionPipeline) currentPipeline).getCenter();
        return cameraToRobotCoords(sampleCenterCameraCoords);
    }

    public double getSampleAngle() {
        if (!(currentPipeline instanceof SampleDetectionPipeline))
            return 0;

        return ((SampleDetectionPipeline) currentPipeline).getAngle();
    }

    public ColorEx getSampleColorRGB() {
        if (!(currentPipeline instanceof SampleDetectionPipeline))
            return new ColorEx(0, 0, 0);

        return ((SampleDetectionPipeline) currentPipeline).getHSV();
    }

    /**
     To convert from camera to Roadrunner coordinate systems, this method does, in order:
        1) moves the origin to the center of the camera, by subtracting *** 1/2HEIGHT ***
            and *** 1/2WIDTH *** from the original coords
        2) newX = -oldY
                &&
           newY = oldX
     */
    private org.firstinspires.ftc.teamcode.Pathing.Math.Point cameraToRobotCoords(Point point) {
        return new org.firstinspires.ftc.teamcode.Pathing.Math.Point(-(point.y - HEIGHT * 0.5), point.x - WIDTH * 0.5);
    }

}
