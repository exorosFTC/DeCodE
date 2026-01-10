package org.firstinspires.ftc.teamcode.CommandBase.Robot.Swerve;

import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.DRIVE_W;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.DRIVE_L;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.R;

import org.firstinspires.ftc.teamcode.CommandBase.Constants.Enums;
import org.firstinspires.ftc.teamcode.Pathing.Math.Point;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

import java.util.Arrays;
import java.util.List;

public class SwerveKinematics {
    private static boolean lockedX = false;



    public static List<SwerveModuleState> robot2wheel(Pose pose) {
        if (lockedX)
            return normalizeSpeeds(Arrays.asList(
                    new SwerveModuleState(0, -Math.PI / 4),     // FR
                    new SwerveModuleState(0, Math.PI / 4),      // FL
                    new SwerveModuleState(0, Math.PI * 3 / 4),  // BL
                    new SwerveModuleState(0, -Math.PI * 3 / 4)  // BR
            ));

        final double vx = pose.x, vy = pose.y, omega = pose.heading;

        // Wheel positions (x_i, y_i) in robot frame: x forward, y left
        // FR: (+L, -W), FL: (+L, +W), BL: (-L, +W), BR: (-L, -W)

        double a = vx + omega * (DRIVE_W / R),
                b = vy + omega * (DRIVE_L / R),
                c = vx - omega * (DRIVE_W / R),
                d = vy - omega * (DRIVE_L / R);

        SwerveModuleState FR = new SwerveModuleState(Math.hypot(a, b), Math.atan2(b, a)); // FR
        SwerveModuleState FL = new SwerveModuleState(Math.hypot(c, b), Math.atan2(b, c)); // FL
        SwerveModuleState BL = new SwerveModuleState(Math.hypot(c, d), Math.atan2(d, c)); // BL
        SwerveModuleState BR = new SwerveModuleState(Math.hypot(a, d), Math.atan2(d, a)); // BR

        return normalizeSpeeds(Arrays.asList(FR, FL, BL, BR));
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
