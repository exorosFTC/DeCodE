package org.firstinspires.ftc.teamcode.Pathing.Math;



import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import java.util.Arrays;
import java.util.List;

public class Transformations {

    public static Pose2d toRoadrunnerPose(Pose pose) {
        if (pose != null)
            return new Pose2d(
                    pose.x,
                    pose.y,
                    Math.toRadians(pose.heading));
        return new Pose2d(0, 0, 0);
    }

    // from radians to and degrees
    public static Pose toCustomPose(Pose2d pose) {
        if (pose != null)
            return new Pose(pose.position.x,
                            pose.position.y,
                            Math.toDegrees(pose.heading.toDouble()));
    return new Pose();
    }



    public static double toCM(double in) { return in * 2.54; }

    public static double toIN(double cm) { return cm / 2.54; }
}
