package org.firstinspires.ftc.teamcode.Hardware.Robot.Components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Intake {
    private final LinearOpMode opMode;
    private final Hardware hardware;

    public Intake(LinearOpMode opMode) {
        hardware = Hardware.getInstance(opMode);

        this.opMode = opMode;
    }

    public void setIntakePower(double p) {

    }
}
