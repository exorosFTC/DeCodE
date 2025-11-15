package org.firstinspires.ftc.teamcode.CommandBase.Util.SensorsEx;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;

public class HuskyLensEx {

    public static class CameraConfig {
        public int imgWidth = 320;
        public int imgHeight = 240;
        public double hFovDeg = 60.0;
        public double vFovDeg = 45.0;
        public double tagSideMeters = 0.050; // 5 cm default

        // Camera pose wrt robot (meters, rad). +x forward, +y left.
        public double camTx = 0.0, camTy = 0.0, camTz = 0.0;
        public double camYawRad = 0.0; // camera yaw vs robot forward
    }

    public static class TagDetection {
        public int id;
        public int x, y, w, h;          // pixels (block center and size)
        public double yawRad;           // +right, radians (camera frame)
        public double pitchRad;         // +down, radians (camera frame)
        public double rangeMeters;      // estimated distance to tag center (meters)
        public long timestampMillis;

        @Override public String toString() {
            return String.format("Tag[id=%d x=%d y=%d w=%d h=%d yaw=%.3f pitch=%.3f R=%.3f m]",
                    id, x, y, w, h, yawRad, pitchRad, rangeMeters);
        }
    }



    private final HuskyLens camera;
    private final CameraConfig config;

    // intrinsics derived from FOV and image size
    private final double cx, cy, fx, fy;



    public HuskyLensEx(HardwareMap hardwareMap) {
        this.camera = hardwareMap.get(HuskyLens.class, "ExoCamera");
        this.config = new CameraConfig();

        this.cx = (config.imgWidth  - 1) * 0.5;
        this.cy = (config.imgHeight - 1) * 0.5;
        this.fx = (config.imgWidth  * 0.5) / Math.tan(Math.toRadians(config.hFovDeg * 0.5));
        this.fy = (config.imgHeight * 0.5) / Math.tan(Math.toRadians(config.vFovDeg * 0.5));
    }



    public void setElementDetection() {
        camera.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        try { Thread.sleep(40); } catch (InterruptedException ignored) {}
    }

    public void setAprilTagDetection() {
        camera.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        try { Thread.sleep(40); } catch (InterruptedException ignored) {}
    }



    public HuskyLens.Block[] blocks() {
        return camera.blocks();
    }

    public HuskyLens.Block[] blocks(int id) { return camera.blocks(id); }

    public HuskyLens.Block largestBlock() {
        HuskyLens.Block[] arr = camera.blocks();
        HuskyLens.Block best = null;
        int bestArea = -1;

        for (HuskyLens.Block b : arr) {
            int area = b.width * b.height;
            if (area > bestArea) { bestArea = area; best = b; }
        }

        return best;
    }

    public HuskyLens.Block largestBlock(int id) {
        HuskyLens.Block[] arr = blocks(id);
        HuskyLens.Block best = null;
        int bestArea = -1;

        for (HuskyLens.Block b : arr) {
            int area = b.width * b.height;
            if (area > bestArea) { bestArea = area; best = b; }
        }

        return best;
    }



    public ArrayList<TagDetection> detectTags() {
        HuskyLens.Block[] arr = camera.blocks();
        long now = System.currentTimeMillis();

        ArrayList<TagDetection> out = new ArrayList<>(arr.length);

        for (HuskyLens.Block b : arr) {
            TagDetection d = toTag(b, now);
            if (d != null) out.add(d);
        }

        return out;
    }

    public TagDetection detectTagById(int id) {
        HuskyLens.Block b = largestBlock(id);
        return (b == null) ? null : toTag(b, System.currentTimeMillis());
    }

    private TagDetection toTag(HuskyLens.Block b, long tsMs) {
        if (b == null || b.width <= 0 || b.height <= 0) return null;

        // Pixel offset from image center
        double du = b.x - cx; // +right
        double dv = b.y - cy; // +down

        // Angles (small-angle pinhole): atan2(offset, focal)
        double yaw = Math.atan2(du, fx);    // rad
        double pitch = Math.atan2(dv, fy);  // rad

        // Range estimate: use pixel height for robustness
        double px = Math.max(1.0, b.height);
        double range = (config.tagSideMeters * fy) / px; // meters

        TagDetection d = new TagDetection();
        d.id = b.id;
        d.x = b.x; d.y = b.y; d.w = b.width; d.h = b.height;
        d.yawRad = yaw; d.pitchRad = pitch; d.rangeMeters = range;
        d.timestampMillis = tsMs;
        return d;
    }



    public double robotBearingRad(TagDetection d) {
        return d.yawRad + config.camYawRad;
    }

    public double[] robotXY(TagDetection d) {
        double br = robotBearingRad(d);
        double x = d.rangeMeters * Math.cos(br) + config.camTx;
        double y = d.rangeMeters * Math.sin(br) + config.camTy;
        return new double[]{x, y};
    }

    /**
     * Estimate robot (x,y) on the field given:
     *  - a tag detection d,
     *  - the tag's known field position (tx, ty) in meters,
     *  - the robot's current heading in field frame (yawFieldRad), from IMU/odo.
     *
     * Math:
     *   field-bearing to tag = yawFieldRad + robotBearingRad(d)
     *   robotXY = tagXY - range * [cos(fieldBearing), sin(fieldBearing)]
     */
    private double[] estimateRobotXYFromKnownTag(TagDetection d,
                                                double tagFieldX, double tagFieldY,
                                                double robotYawFieldRad) {
        double fieldBearing = robotYawFieldRad + robotBearingRad(d);
        double rx = tagFieldX - d.rangeMeters * Math.cos(fieldBearing);
        double ry = tagFieldY - d.rangeMeters * Math.sin(fieldBearing);
        return new double[]{rx, ry};
    }

    public double[] estimateRobotPoseFromKnownTag(TagDetection d,
                                                  double tagFieldX, double tagFieldY,
                                                  double robotYawFieldRad) {
        double[] xy = estimateRobotXYFromKnownTag(d, tagFieldX, tagFieldY, robotYawFieldRad);
        return new double[]{xy[0], xy[1], robotYawFieldRad};
    }

}
