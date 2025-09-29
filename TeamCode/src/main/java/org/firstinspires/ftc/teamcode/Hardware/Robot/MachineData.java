package org.firstinspires.ftc.teamcode.Hardware.Robot;

import org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.Hardware.Constants.SystemConstants;
import org.firstinspires.ftc.teamcode.Hardware.Constants.Interfaces.Enums;

public class MachineData implements Enums {


    public MachineData add(OpMode opModeType) {
        SystemConstants.opModeType = opModeType;
        return this;
    }

    public MachineData getLoopTime(boolean flag) {
        SystemConstants.telemetryAddLoopTime = flag;
        return this;
    }

    public MachineData setAutoOnBlue(boolean flag) {
        SystemConstants.autoOnBlue = flag;
        return this;
    }

    public MachineData setUsingOpenCv(boolean flag) {
        SystemConstants.usingOpenCvCamera = flag;
        return this;
    }

    public MachineData setUsingAprilTag(boolean flag) {
        SystemConstants.usingAprilTagCamera = flag;
        return this;
    }

    public MachineData setUsingAcceleration(boolean flag) {
        DriveConstants.usingAcceleration = flag;
        return this;
    }

    public MachineData setUsingExponentialInput(boolean flag) {
        DriveConstants.usingExponentialInput = flag;
        return this;
    }

    public MachineData setMultithreading(boolean flag) {
        SystemConstants.multithreading = flag;
        return this;
    }


}
