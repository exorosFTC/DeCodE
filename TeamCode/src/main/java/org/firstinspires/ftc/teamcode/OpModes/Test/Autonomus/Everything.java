package org.firstinspires.ftc.teamcode.OpModes.Test.Autonomus;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.IntakeTurret;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.IntakeWrist;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.OuttakeLeftPivot;
import static org.firstinspires.ftc.teamcode.Hardware.Generals.HardwareNames.OuttakeWrist;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Machine;
import org.firstinspires.ftc.teamcode.Hardware.Robot.MachineData;
import org.firstinspires.ftc.teamcode.OpModes.ExoMode;
import org.firstinspires.ftc.teamcode.Pathing.AutoDrive;
import org.firstinspires.ftc.teamcode.Pathing.Math.Pose;

@Config
@Autonomous(name = "ValuesAutoTuner", group = "tuning")
public class Everything extends ExoMode {
    private Machine robot;
    private AutoDrive auto;

    public static double Q = 0.1;
    public static double R = 5;

    private GamepadEx g2;

    @Override
    protected void Init() {
        robot = new Machine()
                .addData(new MachineData()
                        .add(Enums.OpMode.AUTONOMUS)
                        .setAutoOnBlue(false)
                        .getLoopTime(true)
                        .setUsingOpenCv(false)
                        .setUsingAprilTag(false)
                        .setUsingAcceleration(false)
                        .setUsingExponentialInput(false))
                .construct(this);


        g2 = new GamepadEx(gamepad2);
    }

    @Override
    protected void WhenStarted() {
        auto = new AutoDrive(this, robot, new Pose())
                .pause()
                .disableWheelMotors();

        g2 = new GamepadEx(gamepad2);

    }

    @Override
    protected void InitializeThreads() {}

    @Override
    protected void Loop() {
        Pose position = auto.getPosition();
        g2.readButtons();

        robot.drive.left.Q = Q;
        robot.drive.right.Q = Q;

        robot.drive.left.R = R;
        robot.drive.right.R = R;

        if (g2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER))
            auto.setPose(new Pose(0, 0, Math.toRadians(45)));

        //robot.hardware.telemetry.addData("x: ", position.x);
        //robot.hardware.telemetry.addData("y: ", position.y);
        //robot.hardware.telemetry.addData("head: ", position.heading);

        //robot.hardware.telemetry.addData("intake extension: ", robot.system.intake.extension.getPosition());
        //robot.hardware.telemetry.addData("outtake extension: ", robot.system.outtake.extension.getPosition());

        //robot.hardware.telemetry.addData("ultrasonic left (Inch): ", robot.drive.left.getDistance(DistanceUnit.INCH));
        //robot.hardware.telemetry.addData("ultrasonic right (Inch): ", robot.drive.right.getDistance(DistanceUnit.INCH));
        //robot.hardware.telemetry.addData("ultrasonic front (Inch): ", robot.drive.right.getDistance(DistanceUnit.INCH));

        //robot.hardware.telemetry.update();
    }
}
