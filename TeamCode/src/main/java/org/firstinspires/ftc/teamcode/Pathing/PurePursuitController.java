package org.firstinspires.ftc.teamcode.Pathing;

import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.POSE;

import org.firstinspires.ftc.teamcode.CommandBase.Constants.Enums;
import org.firstinspires.ftc.teamcode.Pathing.Math.Point;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

import java.util.ArrayList;
import java.util.List;

public class PurePursuitController {
    private Pose start;
    private Pose end = new Pose();

    private final List<Pose> path = new ArrayList<>();
    private Enums.HeadingMode mode = Enums.HeadingMode.FIXED;

    // default constants. May be overridden
    public double lookahead = 10;
    private double stopRadius = 2.0;

    private int segment = 0;
    private boolean isBusy = false;

    // set the start pose to the current robot pose
    public PurePursuitController() {
        this.start = POSE;
        addPoint(POSE);
    }



    public Pose update() {
        // 0, 1, 2 points: treat as point-to-point (target is the last point)
        if (path.size() < 3 || path.isEmpty()) {
            Pose start = path.get(0);
            Pose goal  = path.get(1);

            if (POSE.distanceTo(goal) <= stopRadius) {
                isBusy = false;
                return new Pose(goal.x, goal.y, goal.heading);
            }
            isBusy = true;

            double heading;
            if (mode == Enums.HeadingMode.LERP) {
                heading = Math.atan2(goal.y - start.y, goal.x - start.x);
            } else if (mode == Enums.HeadingMode.FACE_GOAL) {
                heading = Math.atan2(goal.y - POSE.y, goal.x - POSE.x);
            } else {
                heading = 0;
            }

            return new Pose(goal.x, goal.y, heading);
        }

        // 3+ points: normal pure pursuit
        isBusy = true;

        double r2 = lookahead * lookahead;

        Point bestLookahead = null;
        int bestSeg = segment;
        double bestT = -1;

        for (int i = segment; i < path.size() - 1; i++) {
            Pose a = path.get(i);
            Pose b = path.get(i + 1);

            double dx = b.x - a.x;
            double dy = b.y - a.y;

            double fx = a.x - POSE.x;
            double fy = a.y - POSE.y;

            double A = dx * dx + dy * dy;
            if (A < 1e-9) continue;

            double B = 2.0 * (fx * dx + fy * dy);
            double C = fx * fx + fy * fy - r2;

            double disc = B * B - 4.0 * A * C;
            if (disc < 0) continue;

            double sqrt = Math.sqrt(disc);
            double t1 = (-B - sqrt) / (2.0 * A);
            double t2 = (-B + sqrt) / (2.0 * A);

            double t = -1;
            if (t1 >= 0 && t1 <= 1) t = t1;
            if (t2 >= 0 && t2 <= 1) t = Math.max(t, t2);

            if (t < 0) continue;

            Point candidate = new Point(a.x + dx * t, a.y + dy * t);

            if (i > bestSeg || (i == bestSeg && t > bestT)) {
                bestSeg = i;
                bestT = t;
                bestLookahead = candidate;
            }
        }

        // If we have progressed past intermediate waypoints, advance segment
        while (segment < path.size() - 2 && POSE.distanceTo(path.get(segment + 1)) <= stopRadius) {
            segment++;
        }

        // Finish only on last segment + close to final goal
        Pose finalGoal = path.get(path.size() - 1);
        if (segment >= path.size() - 2 && POSE.distanceTo(finalGoal) <= stopRadius) {
            isBusy = false;
            return new Pose(finalGoal.x, finalGoal.y, finalGoal.heading);
        }

        Point lookaheadPoint;
        double headingT;

        if (bestLookahead != null) {
            segment = bestSeg;
            lookaheadPoint = bestLookahead;
            headingT = bestT;
        } else {
            // fallback: aim at end of current segment
            segment = Math.min(segment, path.size() - 2);
            Pose next = path.get(segment + 1);
            lookaheadPoint = new Point(next.x, next.y);
            headingT = 1.0;
        }

        Pose aSeg = path.get(segment);
        Pose bSeg = path.get(segment + 1);

        return new Pose(
                lookaheadPoint.x,
                lookaheadPoint.y,
                computeHeading(mode, aSeg, bSeg, headingT)
        );
    }



    public PurePursuitController addPoint(Pose point) {
        path.add(point);
        end = point;

        return this;
    }

    public PurePursuitController setMode(Enums.HeadingMode mode) {
        this.mode = mode;
        return this;
    }

    public PurePursuitController setStopRadius(double r) {
        stopRadius = r;
        return this;
    }

    public PurePursuitController setLookahead(double l) {
        lookahead = l;
        return this;
    }



    public boolean isBusy() { return isBusy; }

    public boolean reachedSegment(int num) { return segment == num; }



    private double computeHeading(
            Enums.HeadingMode mode,
            Pose start,
            Pose goal,
            double t
    ) {
        switch (mode) {
            case LERP:
                return lerpAngle(start.heading, goal.heading, t);

            case FACE_GOAL:
                return Math.atan2(goal.y - POSE.y, goal.x - POSE.x);

            case FIXED:
            default:
                return goal.heading;
        }
    }

    private double lerpAngle(double a, double b, double t) {
        t = Math.max(0.0, Math.min(1.0, t));
        double diff = wrapAngle(b - a);
        return wrapAngle(a + diff * t);
    }

    private double wrapAngle(double ang) {
        while (ang <= -Math.PI) ang += 2.0 * Math.PI;
        while (ang >  Math.PI) ang -= 2.0 * Math.PI;
        return ang;
    }
}
