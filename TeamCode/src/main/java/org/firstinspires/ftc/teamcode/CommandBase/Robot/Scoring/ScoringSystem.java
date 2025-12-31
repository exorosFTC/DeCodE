package org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring;

import static org.firstinspires.ftc.teamcode.Pathing.Math.ShootingZoneIntersection.isInShootingZone;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.sun.tools.javac.util.List;

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

    public boolean isIntakeEnabled = true;
    public double[] catchThreshold = new double[]{70, 50, 60};

    public double[] colorDistance = new double[]{-1, -1, -1};
    public NormalizedRGBA colorValues;



    public ScoringSystem(LinearOpMode opMode) {
        this.opMode = opMode;
        this.hardware = Hardware.getInstance(opMode);

        intake = new Intake(opMode);
        indexer = new Indexer(opMode);
        shooter = new Shooter(opMode);
    }



    public void update() {
        updateIntake();
        shooter.update();
    }

    public void updateIntake() {
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

            if (distance < catchThreshold[i]) {
                colorValues = getColorForIndex(i);

                if (colorValues.green > colorValues.red && colorValues.green > colorValues.blue)
                    indexer.elements.set(i, Enums.ArtifactColor.GREEN);
                else indexer.elements.set(i, Enums.ArtifactColor.PURPLE);

            } else if (indexer.elements.get(i) != Enums.ArtifactColor.NONE) {
                indexer.elements.set(i, Enums.ArtifactColor.NONE);
            }
        }


        if (indexer.elements.contains(Enums.ArtifactColor.NONE)) return;

        // when all 3 slots are full, reverse intake & move on
        intake.reverse();
        try { Thread.sleep(400); } catch (InterruptedException e) {}
        intake.off();

        isIntakeEnabled = false;
        indexer.off();
    }

    public void shootSequence() {
        if (!shooter.on) return;

        if (intake.on) {
            isIntakeEnabled = false;
            intake.off();
        }

        if (!indexer.RAPID_FIRE) indexer.indexPattern();

        while (!shooter.ready() && this.opMode.opModeIsActive()) { shooter.update(); try { Thread.sleep(5); } catch (InterruptedException e) {} }
        if (isInShootingZone()) indexer.shoot(3);
        while (indexer.isBusy() && this.opMode.opModeIsActive()) { shooter.update(); try { Thread.sleep(5); } catch (InterruptedException e) {} }


        shooter.off();
        isIntakeEnabled = true;
        indexer.previousLastElement = Enums.ArtifactColor.NONE;

        indexer.zero();
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
