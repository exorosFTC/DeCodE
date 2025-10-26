package org.firstinspires.ftc.teamcode.Hardware.Robot.Scoring.Subsystems;

import static org.firstinspires.ftc.teamcode.Hardware.Constants.HardwareNames.TiltLeftServo;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.HardwareNames.TiltRightServo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Hardware.Robot.Hardware;

public class Tilt {
    private final Hardware hardware;
    private final LinearOpMode opMode;

    private final double on = 0.1 + 66.0 / 125;

    public Tilt(LinearOpMode opMode) {
        this.hardware = Hardware.getInstance(opMode);
        this.opMode = opMode;

        ((ServoImplEx) hardware.servos.get(TiltLeftServo)).setPwmRange(new PwmControl.PwmRange(500, 2500, 5000));
        ((ServoImplEx) hardware.servos.get(TiltRightServo)).setPwmRange(new PwmControl.PwmRange(500, 2500, 5000));
    }

    public void on() {
        hardware.servos.get(TiltLeftServo).setPosition(on);
        hardware.servos.get(TiltRightServo).setPosition(on);
    }

    public void off() {
        // cut the power from both the tilt and shooter hood servos
        hardware.servos.get(TiltLeftServo).getController().pwmDisable();
    }
}
