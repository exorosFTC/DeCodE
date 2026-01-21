package org.firstinspires.ftc.teamcode.CommandBase.Robot;

import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.goalPosition;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.goalPositionBlue;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.DriveConstants.goalPositionRed;

import org.firstinspires.ftc.teamcode.CommandBase.Constants.SystemConstants;

public class SystemData {

    public SystemData add(SystemConstants.OpMode opModeType) {
        SystemConstants.opModeType = opModeType;
        return this;
    }

    public SystemData setAutoOnBlue(boolean flag) {
        SystemConstants.autoOnBlue = flag;
        goalPosition = (flag) ? goalPositionBlue : goalPositionRed;
        return this;
    }
}
