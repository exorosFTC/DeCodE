package org.firstinspires.ftc.teamcode.CommandBase.Robot.Swerve;

import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.DRIVE_W;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.DRIVE_L;

import org.firstinspires.ftc.teamcode.CommandBase.Constants.Enums;
import org.firstinspires.ftc.teamcode.Pathing.Math.Point;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

import java.util.Arrays;
import java.util.List;

public class SwerveKinematics {
    private static boolean lockedX = false;


    /**Performing inverse kinematics to determine each module's state from a drivetrain state*/
    public static List<SwerveModuleState> robot2wheel(Pose pose) {
        final double vx = pose.x, vy = pose.y, omega = pose.heading;

        // Wheel positions (x_i, y_i) in robot frame: x forward, y left
        // FR: (+L, -W), FL: (+L, +W), BL: (-L, +W), BR: (-L, -W)

        double a = vx - omega * (-DRIVE_W),
               b = vy + omega * (DRIVE_L),
               c = vx - omega * (DRIVE_W),
               d = vy + omega * (-DRIVE_L);

        SwerveModuleState FR = new SwerveModuleState(Math.hypot(a, b), Math.atan2(b, a)); // FR
        SwerveModuleState FL = new SwerveModuleState(Math.hypot(c, b), Math.atan2(b, c)); // FL
        SwerveModuleState BL = new SwerveModuleState(Math.hypot(c, d), Math.atan2(d, c)); // BL
        SwerveModuleState BR = new SwerveModuleState(Math.hypot(a, d), Math.atan2(d, a)); // BR

        if (lockedX)
            return normalizeSpeeds(Arrays.asList(
                    new SwerveModuleState(0, -Math.PI / 4),     // FR
                    new SwerveModuleState(0, Math.PI / 4),      // FL
                    new SwerveModuleState(0, Math.PI * 3 / 4),  // BL
                    new SwerveModuleState(0, -Math.PI * 3 / 4)  // BR
            ));

        return normalizeSpeeds(Arrays.asList(FR, FL, BL, BR));
    }

    /**Performing forward kinematics to determine drivetrain's state from module states*/
    public static Pose wheel2robot(List<SwerveModuleState> states) {
        List<Point> vectors = Arrays.asList(
                new Point(states.get(0).getModuleVelocity() * Math.cos(states.get(0).getModuleAngle()),
                          states.get(0).getModuleVelocity() * Math.sin(states.get(0).getModuleAngle())),

                new Point(states.get(1).getModuleVelocity() * Math.cos(states.get(1).getModuleAngle()),
                          states.get(1).getModuleVelocity() * Math.sin(states.get(1).getModuleAngle())),

                new Point(states.get(2).getModuleVelocity() * Math.cos(states.get(2).getModuleAngle()),
                          states.get(2).getModuleVelocity() * Math.sin(states.get(2).getModuleAngle())),

                new Point(states.get(3).getModuleVelocity() * Math.cos(states.get(3).getModuleAngle()),
                          states.get(3).getModuleVelocity() * Math.sin(states.get(3).getModuleAngle()))
        );

        Point vectorPeak = new Point(vectors.get(0).x + vectors.get(1).x + vectors.get(2).x + vectors.get(3).x,
                                     vectors.get(0).y + vectors.get(1).y + vectors.get(2).y + vectors.get(3).y);

        return new Pose(vectorPeak, Math.atan2(vectorPeak.y, vectorPeak.x));
    }


    private static List<SwerveModuleState> normalizeSpeeds(List<SwerveModuleState> states) {
        double max = Double.NEGATIVE_INFINITY;

        for (int i = 0; i < 4; i ++)
            max = Math.max(max, Math.abs(states.get(i).getModuleVelocity()));

        if (max > 1)
            for (int i = 0; i < 4; i ++) {
                states.get(i).setModuleVelocity(states.get(i).getModuleVelocity() / max);
            }

        return states;
    }


    public static void setLockedX(boolean lockedX) { SwerveKinematics.lockedX = lockedX; }

    public static boolean isLockedX() { return lockedX; }
}
