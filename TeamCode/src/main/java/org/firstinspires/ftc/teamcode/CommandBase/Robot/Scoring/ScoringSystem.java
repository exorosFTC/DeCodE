package org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring;

import static org.firstinspires.ftc.teamcode.CommandBase.Constants.SystemConstants.opModeType;
import static org.firstinspires.ftc.teamcode.Pathing.Math.ShootingZoneIntersection.isInShootingZone;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.onbotjava.handlers.file.TemplateFile;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.CommandBase.Constants.Enums;
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

    public ElapsedTime timer;
    public double MIN_LOOPS = 8;

    public boolean shootSorted = false;

    public boolean isIntakeEnabled = true;
    public double[] catchThreshold = new double[]{68, 85, 56};
    public double[] colorDistance = new double[]{-1, -1, -1};
    public double[] loops = new double[]{0, 0, 0};
    public NormalizedRGBA colorValues;



    public ScoringSystem(LinearOpMode opMode) {
        this.opMode = opMode;
        this.hardware = Hardware.getInstance(opMode);

        intake = new Intake(opMode);
        indexer = new Indexer(opMode);
        shooter = new Shooter(opMode);

        timer = new ElapsedTime();
    }




    public void updateIntake() {
        if (timer.milliseconds() < 400) {
            if (hardware.IntakeColor1.getDistance(DistanceUnit.MM) < catchThreshold[0]) {
                colorValues = getColorForIndex(0);

                if (colorValues.green > colorValues.red && colorValues.green > colorValues.blue)
                    indexer.elements.set(0, Enums.ArtifactColor.GREEN);
                else indexer.elements.set(0, Enums.ArtifactColor.PURPLE);
            }
        }

        if (indexer.elements.contains(Enums.ArtifactColor.NONE) && !isIntakeEnabled) isIntakeEnabled = true;
        if (indexer.isBusy() || !indexer.on || !isIntakeEnabled) return;

        colorDistance = new double[]{
                hardware.IntakeColor1.getDistance(DistanceUnit.MM),
                hardware.IntakeColor2.getDistance(DistanceUnit.MM),
                hardware.IntakeColor3.getDistance(DistanceUnit.MM)
        };

        //check all 3 sensors
        for (int i = 0; i < colorDistance.length; i++) {
            double distance = colorDistance[i];

            if (distance < catchThreshold[i] && loops[i] < MIN_LOOPS) { loops[i]+=1; }
            else if (distance < catchThreshold[i] && loops[i] >= MIN_LOOPS) {
                colorValues = getColorForIndex(i);

                if (colorValues.green > colorValues.red && colorValues.green > colorValues.blue)
                    indexer.elements.set(i, Enums.ArtifactColor.GREEN);
                else indexer.elements.set(i, Enums.ArtifactColor.PURPLE);

            } else if (indexer.elements.get(i) != Enums.ArtifactColor.NONE) {
                indexer.elements.set(i, Enums.ArtifactColor.NONE);
            }
        }


        if (indexer.elements.contains(Enums.ArtifactColor.NONE)) return;
        if (opModeType == Enums.OpMode.AUTONOMUS) return;

        // when all 3 slots are full, reverse intake & move on
        intake.reverse();
        try { Thread.sleep(400); } catch (InterruptedException e) {}
        intake.off();
        indexer.off();

        timer.reset();
        isIntakeEnabled = false;
    }

    public void shootSequence() {
        if (!shooter.on) return;
        if (intake.on) { isIntakeEnabled = false; intake.off(); }

        timer.reset();
        while (!shooter.ready() && this.opMode.opModeIsActive() && timer.seconds() < 3) {}

        //if (!isInShootingZone() && opModeType == Enums.OpMode.AUTONOMUS) return;
        indexer.isHome = true;
        indexer.shoot(3);

        shooter.off();
        isIntakeEnabled = true;
        indexer.previousLastElement = Enums.ArtifactColor.NONE;
    }



    private void shootSequenceSorted() {
        if (!shooter.on) return;
        if (intake.on) { isIntakeEnabled = false; intake.off(); }

        timer.reset();
        while (!shooter.ready() && indexer.isBusy() && opMode.opModeIsActive() && timer.seconds() < 5) {}

        //if (!isInShootingZone() && opModeType == Enums.OpMode.AUTONOMUS) return;
        indexer.isHome = true;
        indexer.shoot(3);

        shooter.off();
        isIntakeEnabled = true;
        indexer.previousLastElement = Enums.ArtifactColor.NONE;
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
    public void read() {
        indexer.read();
        shooter.read();
    }

    @Override
    public void write() {
        indexer.write();
        shooter.write();
    }
}
