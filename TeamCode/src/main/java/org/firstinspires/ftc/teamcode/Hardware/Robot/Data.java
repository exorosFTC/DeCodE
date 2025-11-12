package org.firstinspires.ftc.teamcode.Hardware.Robot;

import org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.Hardware.Constants.SystemConstants;
import org.firstinspires.ftc.teamcode.Hardware.Constants.Enums;

public class Data implements Enums {

    public Data add(OpMode opModeType) {
        SystemConstants.opModeType = opModeType;
        return this;
    }

    public Data getLoopTime(boolean flag) {
        SystemConstants.telemetryAddLoopTime = flag;
        return this;
    }

    public Data setAutoOnBlue(boolean flag) {
        SystemConstants.autoOnBlue = flag;
        return this;
    }

    public Data setUsingOpenCv(boolean flag) {
        SystemConstants.usingOpenCvCamera = flag;
        return this;
    }

    public Data setUsingAprilTag(boolean flag) {
        SystemConstants.usingAprilTagCamera = flag;
        return this;
    }

    public Data setUsingFieldCentric(boolean flag) {
        DriveConstants.usingFieldCentric = flag;
        return this;
    }

}
