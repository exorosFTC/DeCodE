package org.firstinspires.ftc.teamcode.CustomPathing.Localizer;

import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.ODOMETRY_X_OFFSET;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.ODOMETRY_Y_OFFSET;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.CommandBase.Util.SensorsEx.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.CustomPathing.Math.Geometry.Pose;


public class PinpointLocalizer {
    private final GoBildaPinpointDriver pinpoint;
    private final ElapsedTime timer;


    public PinpointLocalizer(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "ExoPinpoint");

        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED,       // in front
                                        GoBildaPinpointDriver.EncoderDirection.FORWARD);    // to the left
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setOffsets(ODOMETRY_X_OFFSET, ODOMETRY_Y_OFFSET, DistanceUnit.CM);
        pinpoint.resetPosAndIMU();

        timer = new ElapsedTime();
    }


    public void update() {
        timer.reset();
        pinpoint.update();
        double rawHeading = pinpoint.getHeading(AngleUnit.RADIANS);

        DriveConstants.POSE = new Pose(
                pinpoint.getPosX(DistanceUnit.CM),
                pinpoint.getPosY(DistanceUnit.CM),
                (rawHeading < 0) ? rawHeading + 2 * Math.PI : rawHeading
        );

        DriveConstants.VELOCITY = new Pose(
                pinpoint.getVelX(DistanceUnit.CM),
                pinpoint.getVelY(DistanceUnit.CM),
                pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS)
        );
    }

    public void setPositionEstimate(Pose pose) {
        pinpoint.setPosX(pose.x, DistanceUnit.CM);
        pinpoint.setPosY(pose.y, DistanceUnit.CM);
        pinpoint.setHeading(pose.heading, AngleUnit.RADIANS);
        DriveConstants.POSE = pose;
    }

    public void setOffsets(double xOffset, double yOffset) {
        pinpoint.setOffsets(xOffset, yOffset, DistanceUnit.CM);
    }

}
