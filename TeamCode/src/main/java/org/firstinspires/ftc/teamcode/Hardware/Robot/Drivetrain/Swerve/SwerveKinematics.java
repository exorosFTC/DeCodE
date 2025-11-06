package org.firstinspires.ftc.teamcode.Hardware.Robot.Drivetrain.Swerve;

import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.DRIVE_W;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.DRIVE_L;

import org.firstinspires.ftc.teamcode.Pathing.Math.Point;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

import java.util.Arrays;
import java.util.List;

public class SwerveKinematics {
    protected boolean locked = false;

    /**Performing inverse kinematics to determine each module's state from a drivetrain state*/
    public List<SwerveModuleState> robot2wheel(Pose p) {
        final double vx = p.x, vy = p.y, omega = p.heading;
        final double L = DRIVE_L;   // half length (front/back)
        final double W = DRIVE_W;   // half width  (left/right)

        // Wheel positions (x_i, y_i) in robot frame: x forward, y left
        // FR: (+L, -W), FL: (+L, +W), BL: (-L, +W), BR: (-L, -W)

        // FR
        double vxf = vx - omega * (-W);    // = vx + omega*W
        double vyf = vy + omega * (+L);
        SwerveModuleState FR = new SwerveModuleState(Math.hypot(vxf, vyf), Math.atan2(vyf, vxf));

        // FL
        double vxl = vx - omega * (+W);
        double vyl = vy + omega * (+L);
        SwerveModuleState FL = new SwerveModuleState(Math.hypot(vxl, vyl), Math.atan2(vyl, vxl));

        // BL
        double vxb = vx - omega * (+W);
        double vyb = vy + omega * (-L);
        SwerveModuleState BL = new SwerveModuleState(Math.hypot(vxb, vyb), Math.atan2(vyb, vxb));

        // BR
        double vxr = vx - omega * (-W);    // = vx + omega*W
        double vyr = vy + omega * (-L);
        SwerveModuleState BR = new SwerveModuleState(Math.hypot(vxr, vyr), Math.atan2(vyr, vxr));

        if (locked)
            return normalizeSpeeds(Arrays.asList(
                    new SwerveModuleState(0, Math.PI / 4),     // FR
                    new SwerveModuleState(0, -Math.PI / 4),      // FL
                    new SwerveModuleState(0, -Math.PI * 3 / 4),  // BL
                    new SwerveModuleState(0, Math.PI * 3 / 4)  // BR
            ));

        return normalizeSpeeds(Arrays.asList(FR, FL, BL, BR));
    }


    /**Performing forward kinematics to determine drivetrain's state from module states*/
    public Pose wheel2robot(List<SwerveModuleState> states) {
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



    private List<SwerveModuleState> normalizeSpeeds(List<SwerveModuleState> states) {
        double max = Double.NEGATIVE_INFINITY;

        for (int i = 0; i < 4; i ++)
            max = Math.max(max, Math.abs(states.get(i).getModuleVelocity()));

        if (max > 1)
            for (int i = 0; i < 4; i ++) {
                states.get(i).setModuleVelocity(states.get(i).getModuleVelocity() / max);
            }

        return states;
    }

    protected void setLocked(boolean locked) { this.locked = locked; }

    protected boolean isLocked() { return locked; }
}
