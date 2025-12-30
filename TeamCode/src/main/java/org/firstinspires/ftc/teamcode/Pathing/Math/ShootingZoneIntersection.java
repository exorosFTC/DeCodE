package org.firstinspires.ftc.teamcode.Pathing.Math;

import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.DRIVE_W;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.POSE;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.tileLengthCM;

public class ShootingZoneIntersection {
    //close zone points
    private static final Point T1_A = new Point(3 * tileLengthCM,-3 * tileLengthCM);
    private static final Point T1_B = new Point(0,0);
    private static final Point T1_C = new Point(3 * tileLengthCM,3 * tileLengthCM);

    //far zone points
    private static final Point T2_A = new Point(-3 * tileLengthCM,-tileLengthCM);
    private static final Point T2_B = new Point(-2 * tileLengthCM,0);
    private static final Point T2_C = new Point(-3 * tileLengthCM,tileLengthCM);

    private static final double EPS = 1e-9;



    public static boolean isInShootingZone() {
        return  squareOverlapsTriangle(POSE.point(), DRIVE_W, POSE.heading, T1_A, T1_B, T1_C) ||
                squareOverlapsTriangle(POSE.point(), DRIVE_W, POSE.heading, T2_A, T2_B, T2_C);
    }



    //math
    private static double cross(Point p1, Point p2, Point p3) {
        return (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
    }

    private static boolean pointInTriangle(Point p, Point a, Point b, Point c) {
        double s1 = cross(a, b, p);
        double s2 = cross(b, c, p);
        double s3 = cross(c, a, p);

        boolean hasNeg = (s1 < 0) || (s2 < 0) || (s3 < 0);
        boolean hasPos = (s1 > 0) || (s2 > 0) || (s3 > 0);
        return !(hasNeg && hasPos); //inside or on edge
    }

    private static Point[] squareCorners(Point center, double side, double headingRad) {
        double h = side / 2.0;
        double cos = Math.cos(headingRad);
        double sin = Math.sin(headingRad);

        // ordered around perimeter
        double[][] offs = { {+h, +h}, {+h, -h}, {-h, -h}, {-h, +h} };

        Point[] corners = new Point[4];
        for (int i = 0; i < 4; i++) {
            double dx = offs[i][0], dy = offs[i][1];
            double rx = dx * cos - dy * sin;
            double ry = dx * sin + dy * cos;
            corners[i] = new Point(center.x + rx, center.y + ry);
        }
        return corners;
    }

    private static boolean pointInRotatedSquare(Point p, Point center, double side, double headingRad) {
        double h = side / 2.0;
        double cos = Math.cos(headingRad);
        double sin = Math.sin(headingRad);

        double vx = p.x - center.x;
        double vy = p.y - center.y;

        // rotate by -heading
        double lx =  vx * cos + vy * sin;
        double ly = -vx * sin + vy * cos;

        return Math.abs(lx) <= h && Math.abs(ly) <= h;
    }

    private static int orient(Point a, Point b, Point c) {
        double v = cross(a, b, c);
        if (v > EPS) return 1;
        if (v < -EPS) return -1;
        return 0;
    }

    private static boolean onSegment(Point a, Point b, Point p) {
        return Math.min(a.x, b.x) - EPS <= p.x && p.x <= Math.max(a.x, b.x) + EPS &&
                Math.min(a.y, b.y) - EPS <= p.y && p.y <= Math.max(a.y, b.y) + EPS;
    }

    private static boolean segmentsIntersect(Point a, Point b, Point c, Point d) {
        int o1 = orient(a, b, c);
        int o2 = orient(a, b, d);
        int o3 = orient(c, d, a);
        int o4 = orient(c, d, b);

        // proper intersection
        if (o1 != o2 && o3 != o4) return true;

        // touching/collinear cases
        if (o1 == 0 && onSegment(a, b, c)) return true;
        if (o2 == 0 && onSegment(a, b, d)) return true;
        if (o3 == 0 && onSegment(c, d, a)) return true;
        if (o4 == 0 && onSegment(c, d, b)) return true;

        return false;
    }

    private static boolean squareOverlapsTriangle(Point center, double side, double headingRad,
                                                  Point A, Point B, Point C) {
        Point[] sq = squareCorners(center, side, headingRad);
        Point[] tri = { A, B, C };

        // 1) any square corner inside triangle
        for (Point p : sq) if (pointInTriangle(p, A, B, C)) return true;

        // 2) any triangle vertex inside square
        for (Point p : tri) if (pointInRotatedSquare(p, center, side, headingRad)) return true;

        // 3) any edge intersects
        for (int i = 0; i < 4; i++) {
            Point s1 = sq[i];
            Point s2 = sq[(i + 1) % 4];
            for (int j = 0; j < 3; j++) {
                Point t1 = tri[j];
                Point t2 = tri[(j + 1) % 3];
                if (segmentsIntersect(s1, s2, t1, t2)) return true;
            }
        }

        return false;
    }
}
