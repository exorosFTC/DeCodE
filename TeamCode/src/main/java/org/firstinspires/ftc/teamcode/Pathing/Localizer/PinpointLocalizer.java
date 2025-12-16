package org.firstinspires.ftc.teamcode.Pathing.Localizer;

import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.ODO_UPDATE_RATE_AUTO;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.ODO_UPDATE_RATE_TELEOP;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.SystemConstants.opModeType;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.CommandBase.Constants.Enums;
import org.firstinspires.ftc.teamcode.CommandBase.Util.SensorsEx.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;


public class PinpointLocalizer {
    private GoBildaPinpointDriver pinpoint;
    private ElapsedTime timer;


    public PinpointLocalizer(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "ExoPinpoint");

        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,       // in front
                                        GoBildaPinpointDriver.EncoderDirection.REVERSED);    // to the left
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        //12.6697 -6.6626
        pinpoint.setOffsets(0, 0, DistanceUnit.CM);
        pinpoint.resetPosAndIMU();

        timer = new ElapsedTime();
    }


    public void update() {
        if (timer.milliseconds() < (1000 / (opModeType == Enums.OpMode.AUTONOMUS ? ODO_UPDATE_RATE_AUTO : ODO_UPDATE_RATE_TELEOP))) return;

        timer.reset();
        pinpoint.update();
        double rawHeading = pinpoint.getHeading(AngleUnit.RADIANS);

        DriveConstants.POSE = new Pose(
                pinpoint.getPosX(DistanceUnit.CM),
                pinpoint.getPosY(DistanceUnit.CM),
                (rawHeading < 0) ? rawHeading + 2 * Math.PI : rawHeading
        );
    }

    public Pose getRobotPosition() {
        return DriveConstants.POSE;
    }

    public double getAngle() {
        return DriveConstants.POSE.heading;
    }

    public double getAngularVelocity() {
        return pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
    }

    public void setPositionEstimate(Pose pose) {
        pinpoint.setPosX(pose.x, DistanceUnit.CM);
        pinpoint.setPosY(pose.y, DistanceUnit.CM);
        pinpoint.setHeading(pose.heading, AngleUnit.RADIANS);
        DriveConstants.POSE = pose;
    }

}
