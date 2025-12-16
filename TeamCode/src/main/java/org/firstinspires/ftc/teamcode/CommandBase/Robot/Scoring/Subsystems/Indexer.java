package org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CommandBase.Constants.Enums;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Hardware;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.SystemBase;
import org.firstinspires.ftc.teamcode.CommandBase.Util.SensorsEx.LimelightEx;

import java.util.Arrays;
import java.util.List;

public class Indexer extends SystemBase {
    private final Hardware hardware;
    private final LinearOpMode opMode;
    private final ElapsedTime timer;

    public static final double TICKS_PER_REVOLUTION = 384.5 * 2;
    public static double HOMING_POWER = 0.4; //in the indexing direction
    public static double INDEXING_POWER = 1;
    public static double SHOOTING_POWER = 1;

    private static final int offset = 50;

    public boolean RAPID_FIRE = true;

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
        timer.reset();

        hardware.IndexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.IndexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hardware.IndexerMotor.setPower(-HOMING_POWER);
        while (indexerLimit && opMode.opModeIsActive() && timer.seconds() < 1.6) {}

        target = offset;

        sideswipe(3, true);
        resetEncoder();
        off();
    }

    public void index(int balls) {
        balls = Math.max(1, Math.min(balls, 2)); //indexing 3 spots is useless, limit to 2

        // special check for indexing balls
        if (elements.get(3 - balls) != Enums.ArtifactColor.NONE) target = (int) (target - TICKS_PER_REVOLUTION / 5);

        // index and update the artefact list
        runTarget((int) (target - balls * TICKS_PER_REVOLUTION / 3),
                  elements.get(3 - balls) != Enums.ArtifactColor.NONE ? HOMING_POWER : INDEXING_POWER);
        sideswipe(balls, false);

        // if there is an artefact in the front
        if (elements.get(0) != Enums.ArtifactColor.NONE) {
            // wait for reaching the overshot position
            while (isBusy() && opMode.opModeIsActive()) {}

            // come back so the transfer arm is down
            runTarget(
                    (int) (target + TICKS_PER_REVOLUTION / 5),
                    HOMING_POWER
            );

            // wait to reach and disable the indexer
            while (isBusy() && opMode.opModeIsActive()) {}
            off();
        }
    }

    public void indexPattern() {
        int greenCurrentPos = elements.indexOf(Enums.ArtifactColor.GREEN);
        int greenTargetPos;

        switch (LimelightEx.lastValidRandomization) {
            case LEFT: greenTargetPos = 0;
            case CENTER: greenTargetPos = 1;
            case RIGHT: greenTargetPos = 2;
            default: greenTargetPos = greenCurrentPos;
        }

        int steps = (3 + greenTargetPos - greenCurrentPos) % 3;

        if (steps == 0) return;
        index(steps);

    }

    public void shoot(int balls) {
        balls = Math.max(1, Math.min(balls, 3));

        runTarget(
                (int) (target + balls * TICKS_PER_REVOLUTION * 2 / 3),
                SHOOTING_POWER
        );

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
        else SHOOTING_POWER = 0.5;
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
