package org.firstinspires.ftc.teamcode.Hardware.Robot;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.MecanumConstants;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;

public class MachineData implements Enums {


    public MachineData add(OpMode opModeType) {
        SystemConstants.opModeType = opModeType;
        return this;
    }

    public MachineData add (Color detectionColor) {
        SystemConstants.detectionColor = detectionColor;
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
        MecanumConstants.usingAcceleration = flag;
        return this;
    }

    public MachineData setUsingExponentialInput(boolean flag) {
        MecanumConstants.usingExponentialInput = flag;
        return this;
    }

    public MachineData setMultithreading(boolean flag) {
        SystemConstants.multithreading = flag;
        return this;
    }


}
