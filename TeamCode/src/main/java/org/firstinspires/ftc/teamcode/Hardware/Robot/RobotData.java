package org.firstinspires.ftc.teamcode.Hardware.Robot;

import org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.Hardware.Constants.SystemConstants;
import org.firstinspires.ftc.teamcode.Hardware.Constants.Enums;

public class RobotData implements Enums {


    public RobotData add(OpMode opModeType) {
        SystemConstants.opModeType = opModeType;
        return this;
    }

    public RobotData getLoopTime(boolean flag) {
        SystemConstants.telemetryAddLoopTime = flag;
        return this;
    }

    public RobotData setAutoOnBlue(boolean flag) {
        SystemConstants.autoOnBlue = flag;
        return this;
    }

    public RobotData setUsingOpenCv(boolean flag) {
        SystemConstants.usingOpenCvCamera = flag;
        return this;
    }

    public RobotData setUsingAprilTag(boolean flag) {
        SystemConstants.usingAprilTagCamera = flag;
        return this;
    }

    public RobotData setUsingAcceleration(boolean flag) {
        DriveConstants.usingAcceleration = flag;
        return this;
    }

    public RobotData setUsingExponentialInput(boolean flag) {
        DriveConstants.usingExponentialInput = flag;
        return this;
    }

    public RobotData setUsingFieldCentric(boolean flag) {
        DriveConstants.usingFieldCentric = flag;
        return this;
    }

    public RobotData setMultithreading(boolean flag) {
        SystemConstants.multithreading = flag;
        return this;
    }


}
