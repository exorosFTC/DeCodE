package org.firstinspires.ftc.teamcode.Hardware.Robot.Scoring.Subsystems;

import static org.firstinspires.ftc.teamcode.Hardware.Constants.HardwareNames.IndexerLimit;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.HardwareNames.IndexerMotor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Constants.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Hardware;

import java.util.Arrays;
import java.util.List;

public class Indexer {
    private final Hardware hardware;
    private final LinearOpMode opMode;
    private final ElapsedTime timer;

    public static final double TICKS_PER_REVOLUTION = 384.5 * 0.5;
    public static double HOMING_POWER = 0.5; //in the indexing direction
    public static double INDEXING_POWER = 0.5;

    public static List<Enums.ArtifactColor> elements = Arrays.asList(
            Enums.ArtifactColor.NONE, Enums.ArtifactColor.NONE, Enums.ArtifactColor.NONE
    );

    public Indexer(LinearOpMode opMode) {
        this.hardware = Hardware.getInstance(opMode);
        this.opMode = opMode;

        timer = new ElapsedTime();

        hardware.motors.get(IndexerMotor).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.motors.get(IndexerMotor).setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.motors.get(IndexerMotor).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }


    public void home() {
        timer.reset();

        while (!hardware.digital.get(IndexerLimit).getState() && opMode.opModeIsActive() && timer.seconds() < 1.6) {
            hardware.motors.get(IndexerMotor).setPower(HOMING_POWER);
        }

        off();
        try { Thread.sleep(100); } catch (InterruptedException e) {} //wait for magnet snap
        hardware.motors.get(IndexerMotor).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void index(int balls) {
        balls = Math.max(1, Math.min(balls, 2)); //indexing 3 spots is useless, limit to 2
        double pos = hardware.motors.get(IndexerMotor).getCurrentPosition();

        hardware.motors.get(IndexerMotor).setTargetPosition((int) (pos + balls * TICKS_PER_REVOLUTION / 3));
        hardware.motors.get(IndexerMotor).setPower(INDEXING_POWER);
        hardware.motors.get(IndexerMotor).setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sideswipe(balls, false);

        new Thread(() -> {
            while (hardware.motors.get(IndexerMotor).isBusy() && opMode.opModeIsActive()) {}
            off();
        }).start();
    }

    public void shoot(int balls) {
        balls = Math.max(1, Math.min(balls, 3));
        double pos = hardware.motors.get(IndexerMotor).getCurrentPosition();

        hardware.motors.get(IndexerMotor).setTargetPosition((int) (pos - balls * TICKS_PER_REVOLUTION / 3));
        hardware.motors.get(IndexerMotor).setPower(INDEXING_POWER);
        hardware.motors.get(IndexerMotor).setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sideswipe(balls, true);

        new Thread(() -> {
            while (hardware.motors.get(IndexerMotor).isBusy() && opMode.opModeIsActive()) {}
            home();
        }).start();
    }

    private void sideswipe(int balls, boolean reverse) { //so the name of the robot :)
        if (reverse) {
            switch (balls) {
                case 1: {
                    elements.set(0, elements.get(1));
                    elements.set(1, elements.get(2));
                    elements.set(2, Enums.ArtifactColor.NONE);
                } break;
                case 2: {
                    elements.set(0, elements.get(2));
                    elements.set(1, Enums.ArtifactColor.NONE);
                    elements.set(2, Enums.ArtifactColor.NONE);
                } break;
                case 3: {
                    elements.set(0, Enums.ArtifactColor.NONE);
                    elements.set(1, Enums.ArtifactColor.NONE);
                    elements.set(2, Enums.ArtifactColor.NONE);
                } break;
                default: {}
            }
            return;
        }

        balls = Math.max(1, Math.min(balls, 2));

        switch (balls) {
            case 1: {
                Enums.ArtifactColor copy = elements.get(2);

                elements.set(2, elements.get(1));
                elements.set(1, elements.get(0));
                elements.set(0, copy);
            } break;
            case 2: {
                Enums.ArtifactColor copy = elements.get(0);

                elements.set(0, elements.get(1));
                elements.set(1, elements.get(2));
                elements.set(2, copy);
            } break;
            default: {}
        }
    }


    public void off () { hardware.motors.get(IndexerMotor).setMotorDisable(); }
}
