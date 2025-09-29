package org.firstinspires.ftc.teamcode.Pathing.Path;

import org.firstinspires.ftc.teamcode.Pathing.Math.Point;

import java.util.ArrayList;
import java.util.List;

public class Curve {
    public enum Mode { LINE, CATMULL_ROM, BEZIER }



    public static class Sample {
        public final Point p, tan;

        public Sample(Point p, Point tan) {
            this.p = p;
            this.tan = tan;
        }
    }

    public static class BuiltCurve {
        public final List<Point> points;  // sampled points along curve (world in)
        public final double[] cumulative;     // cumulative arc length (in)
        public final double totalLen;

        public BuiltCurve(List<Point> pts){
            this.points = pts;
            this.cumulative = cumulative(pts);
            this.totalLen = cumulative[cumulative.length - 1];
        }
    }



    public static BuiltCurve build(List<Point> waypoints, Mode mode, boolean loop, int samplesPerSeg){
        if (waypoints.size() < 2) return new BuiltCurve(new ArrayList<>(waypoints));

        switch (mode){
            case CATMULL_ROM: return new BuiltCurve(catmull(waypoints, loop, samplesPerSeg));
            case BEZIER: return new BuiltCurve(bezier(waypoints, loop, samplesPerSeg));
            case LINE:
            default: return new BuiltCurve(new ArrayList<>(waypoints));
        }
    }



    public static Sample sampleAtS(BuiltCurve c, double s, boolean loop){
        if (c.points.isEmpty()) return new Sample(new Point(0,0), new Point(1,0));

        double total = c.totalLen;
        if (total <= 1e-9) return new Sample(c.points.get(0), new Point(1,0));
        if (loop) {
            s = ((s % total) + total) % total;
        } else {
            s = clamp(s, 0, total);
        }

        int i = 0;

        while (i < c.cumulative.length-1 && s > c.cumulative[i + 1]) i++;

        Point a = c.points.get(i);
        Point b = c.points.get((i + 1) % c.points.size());

        double segL = Math.max(1e-9, c.cumulative[i + 1] - c.cumulative[i]);
        double t = (s - c.cumulative[i]) / segL;

        Point p = lerp(a, b, t);
        Point tan = new Point(b.x - a.x, b.y - a.y).norm();

        return new Sample(p, tan);
    }

    public static double curvatureAtS(BuiltCurve c, double s, boolean loop){
        double eps = 0.01; // in (finite difference along arc)

        Point t0 = sampleAtS(c, s - eps, loop).tan;
        Point t1 = sampleAtS(c, s + eps, loop).tan;

        double cross = t0.x * t1.y - t0.y * t1.x;
        double dot   = t0.x * t1.x + t0.y * t1.y;

        double dtheta = Math.atan2(cross, dot);
        return dtheta / (2 * eps);
    }

    public static double closestS(BuiltCurve c, Point p){
        if (c.points.size() < 2) return 0;

        double bestS = 0, bestD2 = Double.POSITIVE_INFINITY;

        for (int i = 0; i < c.points.size() - 1; i++){
            Point a = c.points.get(i), b = c.points.get(i + 1);

            double abx = b.x - a.x, aby = b.y - a.y;
            double ab2 = abx * abx + aby * aby;

            if (ab2 < 1e-12) continue;

            double t = ((p.x - a.x) * abx + (p.y - a.y) * aby) / ab2;
            t = clamp(t,0,1);

            double qx = a.x + abx * t, qy = a.y + aby * t;
            double d2=(qx - p.x) * (qx - p.x) + (qy - p.y) * (qy - p.y);

            if (d2 < bestD2) {
                bestD2 = d2;
                bestS = c.cumulative[i] + t * (c.cumulative[i+1] - c.cumulative[i]);
            }
        }
        return bestS;
    }



    private static List<Point> catmull(List<Point> wps, boolean loop, int m){
        ArrayList<Point> out = new ArrayList<>();
        int N = wps.size(),
            segs= loop ? N : N-1;

        for (int i = 0; i < segs; i++) {
            Point p0 = wps.get((i - 1 + N) % N),
                    p1 = wps.get(i),
                    p2 = wps.get((i + 1) % N),
                    p3 = wps.get((i + 2) % N);

            for (int k = 0; k < m; k++) {
                double t = (double) (k / m);
                out.add(catmullPoint(p0, p1, p2, p3, t));
            }
        }
        if (!loop) out.add(wps.get(N - 1));
        return out;
    }
    private static List<Point> bezier(List<Point> wps, boolean loop, int m){
        ArrayList<Point> out = new ArrayList<>();
        int N = wps.size(),
                segs = loop ? N : N-1;

        Point[] T = new Point[N];
        for (int i = 0; i < N; i++){
            Point prev = wps.get((i - 1 + N) % N),
                    next = wps.get((i + 1) % N);

            double vx = next.x - prev.x,
                    vy = next.y - prev.y;
            boolean interior = loop || (i > 0 && i < N - 1);

            T[i] = new Point(interior ? 0.5 * vx : 0, interior ? 0.5 * vy : 0);
        }

        for (int i = 0; i < segs; i++){
            Point p0 = wps.get(i),
                    p1 = wps.get((i + 1) % N);

            Point c1 = new Point(p0.x + T[i].x / 3.0, p0.y + T[i].y / 3.0);
            Point c2 = new Point(p1.x - T[(i + 1) % N].x / 3.0, p1.y - T[(i + 1) % N].y / 3.0);

            for (int k = 0; k < m; k++) {
                double t = (double) (k / m);
                out.add(bezierPoint(p0, c1, c2, p1, t)); }
        }
        if (!loop) out.add(wps.get(N-1));
        return out;
    }

    private static Point catmullPoint(Point p0, Point p1, Point p2, Point p3, double t){
        double t2 = t*t, t3 = t2 * t;

        double x = 0.5 *
                ((2 * p1.x) +
                (-p0.x + p2.x) * t +
                (2*p0.x - 5*p1.x + 4*p2.x - p3.x) * t2 +
                (-p0.x + 3*p1.x - 3*p2.x + p3.x) * t3);

        double y = 0.5 *
                ((2 * p1.y) +
                (-p0.y + p2.y) * t +
                (2*p0.y - 5*p1.y + 4*p2.y - p3.y) * t2 +
                (-p0.y + 3*p1.y - 3*p2.y + p3.y) * t3);

        return new Point(x,y);
    }
    private static Point bezierPoint(Point p0, Point c1, Point c2, Point p1, double t){
        double u = 1 - t;

        double x = u*u*u*p0.x + 3*u*u*t*c1.x + 3*u*t*t*c2.x + t*t*t*p1.x;
        double y = u*u*u*p0.y + 3*u*u*t*c1.y + 3*u*t*t*c2.y + t*t*t*p1.y;

        return new Point(x, y);
    }

    private static double[] cumulative(List<Point> points){
        double[] cumulative = new double[Math.max(1, points.size())];
        cumulative[0] = 0;

        for (int i = 0; i < points.size() - 1; i++){
            Point a = points.get(i), b = points.get(i+1);
            cumulative[i + 1] = cumulative[i] + Math.hypot(b.x - a.x, b.y - a.y);
        }

        return cumulative;
    }

    private static Point lerp(Point a, Point b, double t) { return new Point(a.x + (b.x-a.x)*t, a.y + (b.y-a.y)*t); }
    private static double clamp(double x, double a, double b){ return Math.max(a, Math.min(b, x)); }
}
