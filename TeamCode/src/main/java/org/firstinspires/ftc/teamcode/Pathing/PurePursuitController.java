package org.firstinspires.ftc.teamcode.Pathing;

import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.POSE;

import org.firstinspires.ftc.teamcode.Hardware.Constants.Enums;
import org.firstinspires.ftc.teamcode.Pathing.Math.Point;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

import java.util.ArrayList;
import java.util.List;

public class PurePursuitController {
    public AutoDrive auto;
    private final List<Point> path = new ArrayList<>();

    public static double lookahead = 10;
    public static int segment = 0;

    private Enums.HeadingMode mode = Enums.HeadingMode.PATH_TANGENT;

    public PurePursuitController(AutoDrive auto) {
        this.auto = auto;
    }



    public void update() {
        if (path.size() < 2) return;

        Point goal = path.get(path.size() - 1);

        // stop when close to goal
        if (POSE.distanceTo(goal) <= 2) {
            auto.driveTo(new Pose(goal.x, goal.y, POSE.heading));
            return;
        }

        double r2 = lookahead * lookahead;
        Point lookaheadPoint = null;

        // simple inline lookahead search
        for (int i = segment; i < path.size() - 1; i++) {
            Point a = path.get(i);
            Point b = path.get(i + 1);
            Point d = new Point(b.x - a.x, b.y - a.y);
            Point f = new Point(a.x - POSE.x, a.y - POSE.y);

            double A = d.x * d.x + d.y * d.y;
            if (A == 0) continue;
            double B = 2 * (f.x * d.x + f.y * d.y);
            double C = f.x * f.x + f.y * f.y - r2;
            double disc = B * B - 4 * A * C;
            if (disc < 0) continue;

            double sqrt = Math.sqrt(disc);
            double t1 = (-B - sqrt) / (2 * A);
            double t2 = (-B + sqrt) / (2 * A);

            // pick the intersection that's within the segment
            if (t1 >= 0 && t1 <= 1) {
                lookaheadPoint = new Point(a.x + d.x * t1, a.y + d.y * t1);
                segment = i;
                break;
            } else if (t2 >= 0 && t2 <= 1) {
                lookaheadPoint = new Point(a.x + d.x * t2, a.y + d.y * t2);
                segment = i;
                break;
            }
        }

        if (lookaheadPoint == null) {
            // fallback: next waypoint
            lookaheadPoint = path.get(Math.min(segment + 1, path.size() - 1));
        }

        // pick heading based on mode
        double heading;
        if (mode == Enums.HeadingMode.PATH_TANGENT) {
            Point a = path.get(segment);
            Point b = path.get(Math.min(segment + 1, path.size() - 1));
            heading = Math.atan2(b.y - a.y, b.x - a.x);
        } else if (mode == Enums.HeadingMode.FACE_GOAL) {
            heading = Math.atan2(goal.y - POSE.y, goal.x - POSE.x);
        } else {
            heading = segment;
        }

        // feed current lookahead to your AutoDrive
        auto.driveTo(new Pose(lookaheadPoint.x, lookaheadPoint.y, heading));
    }



    public void addPoint(Point point) { this.path.add(point); }

    public void setMode(Enums.HeadingMode mode) { this.mode = mode; }

    public void reset() { this.path.clear(); }



    public boolean isBusy() { return this.auto.isBusy(); }

    public Enums.HeadingMode getMode() { return this.mode; }

}



