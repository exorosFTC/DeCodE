package org.firstinspires.ftc.teamcode.Pathing.Path;

import org.firstinspires.ftc.teamcode.Pathing.Math.Point;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

public class ConstrainedFollower {

    public static class Params {
        public double maxVel = 70;      // in/s
        public double maxAccel = 180;   // in/s^2
        public double maxLatAccel = 120;// in/s^2
        public double maxOmega = 3.0;   // rad/s
        public double maxAlpha = 8.0;   // rad/s^2
        public double lookahead = 12;   // in (for heading/goal)
        public boolean loop = false;
        public boolean exactStop = true;
    }
    public static class Output {
        public double vx, vy, omega; // world frame (in/s, rad/s)
        public boolean finished;
    }

    private final Curve.BuiltCurve curve;
    private final Params p;
    private double s = 0;           // arc length progress
    private double sdot = 0;        // along-track speed
    private double omega = 0;       // heading rate
    private double desiredTheta = 0;

    public ConstrainedFollower(Curve.BuiltCurve curve, Params p) {
        this.curve = curve;
        this.p = p;
    }

    public void setSFromPose(Pose pose){
        s = Curve.closestS(curve, new Point(pose.x, pose.y));
    }

    public Output update(Pose robot, double dt, String faceMode, double fixedHeadingRad){
        // ---- speed planning strictly along path ----
        double kappa = Curve.curvatureAtS(curve, s, p.loop);
        double vLimit = p.maxVel;
        if (Math.abs(kappa) > 1e-6) vLimit = Math.min(vLimit, Math.sqrt(Math.max(0, p.maxLatAccel / Math.abs(kappa))));
        if (!p.loop && curve.totalLen > 0) {
            double sToEnd = Math.max(0, curve.totalLen - s);
            vLimit = Math.min(vLimit, Math.sqrt(Math.max(0, 2 * p.maxAccel * sToEnd)));
        }
        double sdotTarget = vLimit;
        sdot += clamp(sdotTarget - sdot, -p.maxAccel * dt, p.maxAccel * dt);
        double sNew = s + sdot * dt;
        if (p.loop) {
            sNew = ((sNew % curve.totalLen) + curve.totalLen) % curve.totalLen;
        } else if (p.exactStop && sNew >= curve.totalLen) {
            sNew = curve.totalLen;
            sdot = 0;
        }
        Curve.Sample now = Curve.sampleAtS(curve, sNew, p.loop);
        Curve.Sample look = Curve.sampleAtS(curve, sNew + Math.max(0, p.lookahead), p.loop);

        // ---- heading target ----
        double thTarget;
        switch (faceMode){
            case "goal":    thTarget = Math.atan2(look.p.y - robot.y, look.p.x - robot.x); break;
            case "fixed":   thTarget = fixedHeadingRad; break;
            case "tangent":
            default:        thTarget = Math.atan2(look.tan.y, look.tan.x); break;
        }
        // smooth & accel-limit ω
        desiredTheta = angleLerp(desiredTheta, thTarget, 0.25);
        double err = wrap(desiredTheta - robot.heading);
        double omegaCmd = clamp(2.5 * err, -p.maxOmega, p.maxOmega);
        omega += clamp(omegaCmd - omega, -p.maxAlpha * dt, p.maxAlpha * dt);

        // ---- world-frame velocity strictly tangent ----
        double vx = now.tan.x * sdot;
        double vy = now.tan.y * sdot;

        // exact final snap (pose on last point, ω -> 0)
        boolean finished = (!p.loop && p.exactStop && Math.abs(sNew - curve.totalLen) < 1e-6);
        s = sNew;

        Output out = new Output();
        out.vx = vx; out.vy = vy; out.omega = omega; out.finished = finished;
        return out;
    }

    public double getS(){ return s; }
    public double getTotal(){ return curve.totalLen; }

    private static double wrap(double a){ while(a>Math.PI) a-=2*Math.PI; while(a<-Math.PI) a+=2*Math.PI; return a; }
    private static double angleLerp(double a,double b,double t){ double d=wrap(b-a); return a + d*t; }
    private static double clamp(double x,double a,double b){ return Math.max(a, Math.min(b, x)); }
}
