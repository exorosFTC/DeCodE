package org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.Subsystems;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CommandBase.Robot.Hardware;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.SystemBase;

public class Tilt extends SystemBase {
    private final Hardware hardware;
    private final LinearOpMode opMode;

    private final double on = 0.1 + 66.0 / 125;

    public Tilt(LinearOpMode opMode) {
        this.hardware = Hardware.getInstance(opMode);
        this.opMode = opMode;
    }



    @Override
    public void read() {}

    @Override
    public void write() {}

    public void reverse() {
        super.on();
        hardware.TiltLeftServo.setPosition(on);
        hardware.TiltRightServo.setPosition(0);
    }

    @Override
    public void on() {
        super.on();
        hardware.TiltLeftServo.setPosition(0);
        hardware.TiltRightServo.setPosition(on);
    }

    @Override
    public void off() {
        super.off();

        // cut the power from both the tilt and shooter hood servos
        hardware.TiltLeftServo.getController().pwmDisable();
    }
}
