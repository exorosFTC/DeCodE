package org.firstinspires.ftc.teamcode.Hardware.Robot.Localizer.IMU;


import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.LOGO_DIRECTION;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.USB_DIRECTION;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.IMU_Name;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Localizer;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

/**Test class for field-centric drive before mounting odometry wheels
 * Used strictly for the HEADING value**/
public class SketchyIMU implements Localizer {
    public IMU it;


    private double imuAngle = 0.0;
    private double imuVelocity = 0.0;


    public SketchyIMU getInstance() { return this; }

    public SketchyIMU(LinearOpMode opMode) {
        it = opMode.hardwareMap.get(IMU.class, IMU_Name);
        it.initialize(new IMU.Parameters(
                    new RevHubOrientationOnRobot(LOGO_DIRECTION, USB_DIRECTION)));

        reset();
    }

    public PoseVelocity2d update() {
        Orientation angles = it.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);

        imuAngle = normalizeAngle(angles.thirdAngle);
        imuVelocity = it.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
        return null;
    }


    /**The only useful value from here**/
    public double getAngle(AngleUnit unit) {
        switch (unit) {
            case RADIANS: { return imuAngle; }
            case DEGREES: { return fromRadiansToDegrees(imuAngle); }
            default: { return 0; }
        }
    }

    public double getVelocity(AngleUnit unit) {
        switch (unit) {
            case RADIANS: { return imuVelocity; }
            case DEGREES: { return fromRadiansToDegrees(imuVelocity); }
            default: { return 0; }
        }
    }

    public Pose getRobotPosition() { return new Pose(0, 0, getAngle(AngleUnit.RADIANS)); }

    public Pose getRobotVelocity() { return new Pose(0, 0, getVelocity(AngleUnit.RADIANS)); }


    public double normalizeAngle(double angle) {
        return angle + Math.PI;
    }

    public void reset() { it.resetYaw(); }

    public void setPositionEstimate(Pose newPoseEstimate) { }
}
