package org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.Subsystems;

import static org.firstinspires.ftc.teamcode.CommandBase.Constants.SystemConstants.lastValidRandomization;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.SystemConstants.opModeType;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandBase.Constants.SystemConstants;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Hardware;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.SystemBase;

import java.util.Arrays;
import java.util.List;

public class Indexer extends SystemBase {
    public enum Artifact {
        PURPLE,
        GREEN,
        NONE,
    }

    private final Hardware hardware;
    private final LinearOpMode opMode;
    private final ElapsedTime timer;

    public static final double TICKS_PER_REVOLUTION = 336;
    public static double HOMING_POWER = 0.2; //in the indexing direction
    public static double INDEXING_POWER = 0.3;

    public static final int microAdjustValue = 15;
    public int offset = 0;

    public boolean isHome = true;
    public boolean isIndexing = false;
    public static boolean sorted = false;

    public int target = 0;
    public int indexerPosition = 0;
    public boolean indexerLimit = true;

    public static List<Artifact> elements = Arrays.asList(
            Artifact.NONE,
            Artifact.NONE,
            Artifact.NONE);



    public Indexer(LinearOpMode opMode) {
        this.hardware = Hardware.getInstance(opMode);
        this.opMode = opMode;

        timer = new ElapsedTime();
    }



    public void home() {
        hardware.IndexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.IndexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // home in the shooting direction
        hardware.IndexerMotor.setPower(HOMING_POWER); timer.reset();

        while (timer.milliseconds() < 200 || !indexerLimit && opMode.opModeIsActive()) {}
        while (indexerLimit && opMode.opModeIsActive() && timer.seconds() < 3) {}

        if (timer.seconds() < 3) isHome = true;

        target = 0;

        sideswipe(3, true);
        resetEncoder();

        runTarget(target, 1);
    }



    public void index(int balls) {
        isIndexing = true;
        balls = Math.max(1, Math.min(balls, 2)); //indexing 3 spots is useless, limit to 2

        // special check for indexing balls
        if (elements.get(3 - balls) != Artifact.NONE) target = (int) (target - TICKS_PER_REVOLUTION / 6);

        // index and update the artefact list
        target -= (int) (balls * TICKS_PER_REVOLUTION / 3);
        offset += (int) (balls * TICKS_PER_REVOLUTION / 3);

        sideswipe(balls, false);

        hardware.IndexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.IndexerMotor.setPower(-INDEXING_POWER);
        while (indexerPosition > target && opMode.opModeIsActive()) {}
        hardware.IndexerMotor.setPower(0);

        try {Thread.sleep(300);} catch (InterruptedException e) {}

        // if there is an artefact in the front
        if (elements.get(0) != Artifact.NONE) {
            // come back so the transfer arm is down
            target += (int) (TICKS_PER_REVOLUTION / 6);

            hardware.IndexerMotor.setPower(0.25);
            while (indexerPosition < target && opMode.opModeIsActive()) {}
            hardware.IndexerMotor.setPower(0);
        }

        runTarget(target, 1);
        isIndexing = false;
    }

    public void indexPattern() {
        int greenCurrentPos = elements.indexOf(Artifact.GREEN);
        int greenTargetPos = -1;

        switch (lastValidRandomization) {
            case LEFT: greenTargetPos = 0; break;
            case CENTER: greenTargetPos = 1; break;
            case RIGHT: greenTargetPos = 2; break;
        }

        int steps = (3 + greenTargetPos - greenCurrentPos) % 3;
        sorted = true;

        if (steps == 0) return;
        index(steps);

    }



    public void microAdjust(boolean reverse) {
        offset += reverse ? -microAdjustValue : microAdjustValue;

        runTarget(target + (reverse ? 1 : -1) * microAdjustValue,
                  INDEXING_POWER);
    }

    public void manual(double value, double sensitivity) {
        if (Math.abs(value) < 0.01) return;

        offset += (int) (value * TICKS_PER_REVOLUTION * sensitivity);
        runTarget( target + (int) (value * TICKS_PER_REVOLUTION * sensitivity), 1);
    }




    public void shoot(int balls) {
        shoot(balls, -1);
    }

    public void shoot(int balls, double overridePower) {
        balls = Math.max(1, Math.min(balls, 3));


        // set raw power for a faster transfer
        hardware.IndexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.IndexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.on = true;
        this.target = (int) (target + offset + balls * TICKS_PER_REVOLUTION / 3 * (Indexer.sorted ? 2 : 1));

        timer.reset();
        while (indexerPosition < this.target && timer.seconds() < 2 && opMode.opModeIsActive()) {
            hardware.IndexerMotor.setPower(sorted ? (this.target - indexerPosition > TICKS_PER_REVOLUTION ? 0.4 : Shooter.sample.transferPower) : (overridePower == -1 ? Shooter.sample.transferPower : overridePower));
        }

        // use PID for holding the intake position, after overshooting
        runTarget(target, 1);

        sideswipe(balls, true);
        if (balls == 3) offset = 0;

        sorted = false;
    }




    public void sideswipe(int balls, boolean reverse) { //so the name of the robot :)
        if (reverse) {
            switch (balls) {
                case 1: {
                    elements.set(0, elements.get(1));
                    elements.set(1, elements.get(2));
                    elements.set(2, Artifact.NONE);
                } break;
                case 2: {
                    elements.set(0, elements.get(2));
                    elements.set(1, Artifact.NONE);
                    elements.set(2, Artifact.NONE);
                } break;
                case 3: {
                    elements.set(0, Artifact.NONE);
                    elements.set(1, Artifact.NONE);
                    elements.set(2, Artifact.NONE);
                } break;
                default: {}
            }

            return;
        }

        balls = Math.max(1, Math.min(balls, 2));

        switch (balls) {
            case 1: {
                Artifact copy = elements.get(2);

                elements.set(2, elements.get(1));
                elements.set(1, elements.get(0));
                elements.set(0, copy);
            } break;
            case 2: {
                Artifact copy = elements.get(0);

                elements.set(0, elements.get(1));
                elements.set(1, elements.get(2));
                elements.set(2, copy);
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



    public boolean isBusy() { return  isBusy(5);}

    public boolean isBusy(int threshold) { return Math.abs(target - indexerPosition) > threshold; }

    public void preload() {
        resetEncoder();
        runTarget(target, 1);

        elements.set(0, Artifact.GREEN);
        elements.set(1, Artifact.PURPLE);
        elements.set(2, Artifact.PURPLE);
    }
}
