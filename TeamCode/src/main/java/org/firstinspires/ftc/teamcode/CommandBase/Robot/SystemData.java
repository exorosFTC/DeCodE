package org.firstinspires.ftc.teamcode.CommandBase.Robot;

import org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.CommandBase.Constants.SystemConstants;
import org.firstinspires.ftc.teamcode.CommandBase.Constants.Enums;

public class SystemData implements Enums {

    public SystemData add(OpMode opModeType) {
        SystemConstants.opModeType = opModeType;
        return this;
    }

    public SystemData getLoopTime(boolean flag) {
        SystemConstants.telemetryAddLoopTime = flag;
        return this;
    }

    public SystemData setAutoOnBlue(boolean flag) {
        SystemConstants.autoOnBlue = flag;
        return this;
    }
}
