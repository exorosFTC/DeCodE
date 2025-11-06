package org.firstinspires.ftc.teamcode.Pathing.Localizer;

import static org.firstinspires.ftc.teamcode.Hardware.Constants.HardwareNames.PinpointName;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.Hardware.Util.SensorsEx.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;


public class PinpointLocalizer {

    private GoBildaPinpointDriver pinpoint;
    private Pose currentPose = new Pose(0, 0, 0);


    public PinpointLocalizer(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PinpointName);

        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,       // in front
                                        GoBildaPinpointDriver.EncoderDirection.FORWARD);    // to the left
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setOffsets(12.6697, -6.6626, DistanceUnit.CM);
        pinpoint.resetPosAndIMU();
    }


    public void update() {
        pinpoint.update();
        double rawHeading = pinpoint.getHeading(AngleUnit.RADIANS);

        currentPose = new Pose(
                pinpoint.getPosX(DistanceUnit.CM),
                pinpoint.getPosY(DistanceUnit.CM),
                (rawHeading < 0) ? rawHeading + 2 * Math.PI : rawHeading
        );
    }

    public Pose getRobotPosition() {
        return currentPose;
    }

    public double getAngle() {
        return currentPose.heading;
    }

    public double getAngularVelocity() {
        return pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
    }

    public void setPositionEstimate(Pose pose) {
        pinpoint.setPosX(pose.x, DistanceUnit.CM);
        pinpoint.setPosY(pose.y, DistanceUnit.CM);
        pinpoint.setHeading(-pose.heading, AngleUnit.RADIANS);
        currentPose = pose;
    }

}
