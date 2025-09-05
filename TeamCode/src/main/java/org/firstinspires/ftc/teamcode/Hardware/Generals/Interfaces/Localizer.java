package org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces;


import com.acmerobotics.roadrunner.PoseVelocity2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

/**Localizer interface for smoother code*/
public interface Localizer {
    double getAngle(AngleUnit unit);

    PoseVelocity2d update();

    Pose getRobotPosition();

    void setPositionEstimate(Pose newPose);

    default double fromRadiansToDegrees(double val) {
        return val * 180 / Math.PI;
    }

    default double fromDegreesToRadians(double val) {
        return val *Math.PI / 180;
    }
}
