package org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.Subsystems;

import static org.firstinspires.ftc.teamcode.CommandBase.Constants.SystemConstants.lastValidRandomization;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandBase.Constants.Enums;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Hardware;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.SystemBase;

import java.util.Arrays;
import java.util.List;

public class Indexer extends SystemBase {
    private final Hardware hardware;
    private final LinearOpMode opMode;
    private final ElapsedTime timer;

    public static final double TICKS_PER_REVOLUTION = 336;
    public static double HOMING_POWER = 0.2; //in the indexing direction
    public static double INDEXING_POWER = 0.4;
    public static double SHOOTING_POWER = 1;

    private static final int offset = 0;

    public boolean RAPID_FIRE = true;
    public boolean isHome = true;

    public int target = offset;
    public int indexerPosition = 0;
    public boolean indexerLimit = true;

    public List<Enums.ArtifactColor> elements = Arrays.asList(
            Enums.ArtifactColor.NONE,
            Enums.ArtifactColor.NONE,
            Enums.ArtifactColor.NONE);
    public Enums.ArtifactColor previousLastElement = Enums.ArtifactColor.NONE;



    public Indexer(LinearOpMode opMode) {
        this.hardware = Hardware.getInstance(opMode);
        this.opMode = opMode;

        timer = new ElapsedTime();
    }



    public void home() {
        hardware.IndexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.IndexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // home in the shooting direction
        if (indexerLimit) { hardware.IndexerMotor.setPower(HOMING_POWER); timer.reset(); }
        while (indexerLimit && opMode.opModeIsActive() && timer.seconds() < 3) {}
        if (timer.seconds() < 3) isHome = true;

        target = offset;

        sideswipe(3, true);
        resetEncoder();
        off();
    }



    public void index(int balls) {
        balls = Math.max(1, Math.min(balls, 2)); //indexing 3 spots is useless, limit to 2

        // special check for indexing balls
        if (elements.get(3 - balls) != Enums.ArtifactColor.NONE) target = (int) (target - TICKS_PER_REVOLUTION / 4.6);

        // index and update the artefact list
        runTarget((int) (target - balls * TICKS_PER_REVOLUTION / 3), INDEXING_POWER);
        sideswipe(balls, false);

        // if there is an artefact in the front
        if (elements.get(0) != Enums.ArtifactColor.NONE) {
            // wait for reaching the overshot position
            hardware.IndexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            hardware.IndexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            hardware.IndexerMotor.setPower(-INDEXING_POWER * 13.5 / hardware.batteryVoltage);
            while (indexerPosition > target && opMode.opModeIsActive()) {}
            off();

            // come back so the transfer arm is down

            target = (int) (target + TICKS_PER_REVOLUTION / 4.9);
            hardware.IndexerMotor.setPower(0.16 * 13.5 / hardware.batteryVoltage);

            while (indexerPosition < target && opMode.opModeIsActive()) { }
            off();
        }
    }

    public void indexPattern() {
        int greenCurrentPos = elements.indexOf(Enums.ArtifactColor.GREEN);
        int greenTargetPos;

        switch (lastValidRandomization) {
            case LEFT: greenTargetPos = 0;
            case CENTER: greenTargetPos = 1;
            case RIGHT: greenTargetPos = 2;
            default: greenTargetPos = 0;
        }

        int steps = (3 + greenTargetPos - greenCurrentPos) % 3;
        hardware.telemetry.addData("steps", steps);

        if (steps == 0) return;
        index(steps);

    }

    public void microAdjust(boolean reverse) {
        runTarget(target + (reverse ? 1 : -1) * 20,
                  INDEXING_POWER);
    }

    public void zero() {
        runTarget(0,
                SHOOTING_POWER);

        timer.reset();
        while (isBusy() && opMode.opModeIsActive() && timer.seconds() < 3) {}
    }

    public void shoot(int balls) {
        balls = Math.max(1, Math.min(balls, 3));


        timer.reset();
        hardware.IndexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.IndexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.target = (int) (target + balls * TICKS_PER_REVOLUTION * 2 / 3 );
        this.on = true;

        hardware.IndexerMotor.setPower(SHOOTING_POWER * 13.5 / hardware.batteryVoltage);
        while (indexerPosition < target && opMode.opModeIsActive() && timer.seconds() < 2) {}
        hardware.IndexerMotor.setPower(0);

        sideswipe(balls, true);
    }



    public void sideswipe(int balls, boolean reverse) { //so the name of the robot :)
        if (reverse) {
            switch (balls) {
                case 1: {
                    elements.set(0, elements.get(1));
                    elements.set(1, elements.get(2));
                    elements.set(2, Enums.ArtifactColor.NONE);

                    previousLastElement = elements.get(0);
                } break;
                case 2: {
                    elements.set(0, elements.get(2));
                    elements.set(1, Enums.ArtifactColor.NONE);
                    elements.set(2, Enums.ArtifactColor.NONE);

                    previousLastElement = elements.get(0);
                } break;
                case 3: {
                    elements.set(0, Enums.ArtifactColor.NONE);
                    elements.set(1, Enums.ArtifactColor.NONE);
                    elements.set(2, Enums.ArtifactColor.NONE);

                    previousLastElement = Enums.ArtifactColor.NONE;
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

                previousLastElement = elements.get(0);
            } break;
            case 2: {
                Enums.ArtifactColor copy = elements.get(0);

                elements.set(0, elements.get(1));
                elements.set(1, elements.get(2));
                elements.set(2, copy);

                previousLastElement = elements.get(1);
            } break;
            default: {}
        }
    }



    public void runTarget(int target, double power) {
        this.target = target;

        hardware.IndexerMotor.setTargetPosition(target);
        hardware.IndexerMotor.setPower(power);
        hardware.IndexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        on = true;
    }

    public void resetEncoder() {
        hardware.IndexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setRapidFire(boolean flag) {
        this.RAPID_FIRE = flag;

        if (flag) SHOOTING_POWER = 1;
        else SHOOTING_POWER = 0.4;
    }




    @Override
    public void read() {
        indexerPosition = hardware.IndexerMotor.getCurrentPosition();
        indexerLimit = hardware.IndexerLimit.getState();
    }

    @Override
    public void write() {}

    @Override
    public void on() {
        super.on();

        hardware.IndexerMotor.setTargetPosition(target);
        hardware.IndexerMotor.setPower(INDEXING_POWER);
        hardware.IndexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void off () {
        super.off();
        hardware.IndexerMotor.setMotorDisable();
    }



    public boolean isBusy() { return Math.abs(target - indexerPosition) > 5;}
}
