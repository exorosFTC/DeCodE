package org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CommandBase.Robot.Hardware;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.SystemBase;

public class Lift extends SystemBase {
    private final Hardware hardware;
    private final LinearOpMode opMode;

    public boolean breakLeft, breakRight;

    public Lift(LinearOpMode opMode) {
        this.hardware = Hardware.getInstance(opMode);
        this.opMode = opMode;
    }


    @Override
    public void read() {
        if (!on) return;

        breakLeft = hardware.LeftBreakBeam.getState();
        breakRight = hardware.RightBreakBeam.getState();
    }

    @Override
    public void write() {}

    public void update() {
        if (!on) return;

        if (!breakLeft) {
            hardware.LeftBack_lift.setPower(1);
            hardware.LeftFront_lift.setPower(-1);
        } else {
            hardware.LeftBack_lift.setPower(0);
            hardware.LeftFront_lift.setPower(0);
        }

        if (!breakRight) {
            hardware.RightBack_lift.setPower(-1);
            hardware.RightFront_lift.setPower(1);
        } else {
            hardware.RightBack_lift.setPower(0);
            hardware.RightFront_lift.setPower(0);
        }
    }

    public boolean isBusy() {
        return !(breakLeft && breakRight);
    }
}
