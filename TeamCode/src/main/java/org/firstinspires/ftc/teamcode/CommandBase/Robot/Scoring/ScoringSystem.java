package org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring;

import static org.firstinspires.ftc.teamcode.CommandBase.Constants.SystemConstants.opModeType;
import static org.firstinspires.ftc.teamcode.CommandBase.Constants.SystemConstants.soloDrive;
import static org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.Subsystems.Indexer.elements;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.CommandBase.Constants.SystemConstants;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Hardware;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.SystemBase;

public class ScoringSystem extends SystemBase {
    private final Hardware hardware;
    private final LinearOpMode opMode;

    public Intake intake;
    public Indexer indexer;
    public Shooter shooter;

    public boolean isArmUp = false;
    public boolean isShooting = false;

    private double armDown = 0.15, // 0.13
                   armUp = 0.29;   // 0.34

    public ElapsedTime timer;
    public double MIN_LOOPS = 8;

    public double[] catchThreshold = new double[]{68, 69, 52};
    public double[] colorDistance = new double[]{-1, -1, -1};
    public double[] loops = new double[]{0, 0, 0};
    public NormalizedRGBA colorValues;

    public boolean isIndexerFull = false;



    public ScoringSystem(LinearOpMode opMode) {
        this.opMode = opMode;
        this.hardware = Hardware.getInstance(opMode);

        intake = new Intake(opMode);
        indexer = new Indexer(opMode);
        shooter = new Shooter(opMode);

        timer = new ElapsedTime();
    }




    public void updateIntake(boolean ignore) {
        if (indexer.isBusy() || !indexer.on || !intake.on || !isArmUp) return;

        colorDistance = new double[]{
                hardware.IntakeColor1.getDistance(DistanceUnit.MM),
                hardware.IntakeColor2.getDistance(DistanceUnit.MM),
                hardware.IntakeColor3.getDistance(DistanceUnit.MM)
        };

        isIndexerFull = (colorDistance[0] <= catchThreshold[0]
                                            &&
                         colorDistance[1] <= catchThreshold[1]
                                            &&
                         colorDistance[2] <= catchThreshold[2]);

        //check all 3 sensors
        for (int i = 0; i < colorDistance.length; i++) {
            double distance = colorDistance[i];

            if (distance < catchThreshold[i] && loops[i] < MIN_LOOPS) { loops[i]+=1; }
            else if (distance < catchThreshold[i] && loops[i] >= MIN_LOOPS) {
                colorValues = getColorForIndex(i);

                if (colorValues.green > colorValues.red && colorValues.green > colorValues.blue)
                    elements.set(i, Indexer.Artifact.GREEN);
                else elements.set(i, Indexer.Artifact.PURPLE);

            } else if (elements.get(i) != Indexer.Artifact.NONE) {
                elements.set(i, Indexer.Artifact.NONE);
            }
        }


        if (elements.contains(Indexer.Artifact.NONE)
                                ||
            opModeType == SystemConstants.OpMode.AUTONOMOUS
                                ||
                              ignore) return;


        // when all 3 slots are full, reverse intake & move on
        intake.reverse();
        timer.reset();

        while (opMode.opModeIsActive() && timer.milliseconds() < 400) {
            if (hardware.IntakeColor1.getDistance(DistanceUnit.MM) < catchThreshold[0]) {
                colorValues = getColorForIndex(0);

                if (colorValues.green > colorValues.red && colorValues.green > colorValues.blue)
                    elements.set(0, Indexer.Artifact.GREEN);
                else elements.set(0, Indexer.Artifact.PURPLE);
            }
        }

        intake.off();
    }

    public void shootSequence() {
        if (!shooter.on) return;
        isShooting = true;

        if (intake.on) intake.off();
        if (isArmUp) load();

        timer.reset();
        while (!shooter.ready() && this.opMode.opModeIsActive() && timer.seconds() < 3) {}

        if (opModeType == SystemConstants.OpMode.TELE_OP) {
            timer.reset();
            while (this.opMode.opModeIsActive() && timer.milliseconds() < 100) {}
        }

        indexer.shoot(3);

        setTransferArm(true);
        isShooting = false;
        if (soloDrive) shooter.off();
    }

    public void setTransferArm(boolean up) { setTransferArm(up, true); }

    public void setTransferArm(boolean up, boolean updateValue) {
        if (updateValue) isArmUp = up;

        if (up) hardware.TransferArmServo.setPosition(armUp);
        else hardware.TransferArmServo.setPosition(armDown);
    }

    public void load() {
        if (!isArmUp) return;
        indexer.microAdjust(false);

        timer.reset();
        while (indexer.isBusy(3) && timer.seconds() < 1.5 && opMode.opModeIsActive()) {}

        setTransferArm(false, true);
        indexer.microAdjust(true);
    }


    private NormalizedRGBA getColorForIndex(int i) {
        switch (i) {
            case 0: return hardware.IntakeColor1.getNormalizedColors();
            case 1: return hardware.IntakeColor2.getNormalizedColors();
            case 2: return hardware.IntakeColor3.getNormalizedColors();
            default: return null;
        }
    }


    @Override
    public void read() { indexer.read(); shooter.read(); }

    @Override
    public void write() { shooter.write(); }
}


