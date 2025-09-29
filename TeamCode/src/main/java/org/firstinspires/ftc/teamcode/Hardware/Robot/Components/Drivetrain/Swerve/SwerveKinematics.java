package org.firstinspires.ftc.teamcode.Hardware.Robot.Components.Drivetrain.Swerve;

import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.DRIVE_W;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.DRIVE_L;
import static org.firstinspires.ftc.teamcode.OpModes.Test.Autonomus.Everything.R;

import org.firstinspires.ftc.teamcode.Pathing.Math.Point;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

import java.util.Arrays;
import java.util.List;

abstract class SwerveKinematics {
    protected boolean locked = false;

    /**Performing inverse kinematics to determine each module's state from a drivetrain state*/
    public List<SwerveModuleState> robot2wheel(Pose p) {
        double x = p.x, y = p.y, heading = p.heading;
        List<SwerveModuleState> states;

        double  a = x - heading * (DRIVE_L / R),
                b = x + heading * (DRIVE_L / R),
                c = y - heading * (DRIVE_W / R),
                d = y + heading * (DRIVE_W / R);

        // angle in range [-π, +π]
        if (locked) {
            states = Arrays.asList(
                    new SwerveModuleState(0, Math.PI / 4),     // FR lock
                    new SwerveModuleState(0, -Math.PI / 4),    // FL lock
                    new SwerveModuleState(0, Math.PI / 4),     // BL lock
                    new SwerveModuleState(0, -Math.PI / 4));   // BR lock
        } else {
            states = Arrays.asList(
                    new SwerveModuleState(Math.hypot(b, c), Math.atan2(b, c)),     // FR speeds
                    new SwerveModuleState(Math.hypot(b, d), Math.atan2(b, d)),     // FL speeds
                    new SwerveModuleState(Math.hypot(a, d), Math.atan2(a, d)),     // BL speeds
                    new SwerveModuleState(Math.hypot(a, c), Math.atan2(a, c)));    // BR speeds
        }

        return normalizeSpeeds(states);
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
