package org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.CommandBase.Constants.Enums;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Hardware;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.Scoring.Subsystems.Tilt;
import org.firstinspires.ftc.teamcode.CommandBase.Robot.SystemBase;

public class ScoringSystem extends SystemBase {
    private final Hardware hardware;
    private final LinearOpMode opMode;

    public Intake intake;
    public Indexer indexer;
    public Shooter shooter;
    public Tilt tilt;

    public boolean isIntakeEnabled = true;

    private double lastDistance = 1000;
    private int loopCount = 0;

    public double colorDistance;
    public NormalizedRGBA colorValues;

    public ScoringSystem(LinearOpMode opMode) {
        this.opMode = opMode;
        this.hardware = Hardware.getInstance(opMode);

        intake = new Intake(opMode);
        indexer = new Indexer(opMode);
        shooter = new Shooter(opMode);
        tilt = new Tilt(opMode);
    }



    public void update() {
        updateIntake();
        shooter.update();
    }

    public void updateIntake() {
        colorDistance = hardware.IntakeColor.getDistance(DistanceUnit.MM);

        if (Math.abs(colorDistance - lastDistance) < 2) loopCount += 1;
        else {
            loopCount = 0;
            lastDistance = colorDistance;
        }



        if (indexer.elements.contains(Enums.ArtifactColor.NONE) && !isIntakeEnabled) isIntakeEnabled = true;

        if (colorDistance > 78 && indexer.elements.get(0) != Enums.ArtifactColor.NONE) indexer.elements.set(0, Enums.ArtifactColor.NONE);
        if (colorDistance > 78 || indexer.isBusy() || !indexer.on || !isIntakeEnabled || loopCount < 4) return;



        colorValues = hardware.IntakeColor.getNormalizedColors();

        if (colorValues.green > colorValues.red && colorValues.green > colorValues.blue) indexer.elements.set(0, Enums.ArtifactColor.GREEN);
        else indexer.elements.set(0, Enums.ArtifactColor.PURPLE);

        if (indexer.elements.get(2) != Enums.ArtifactColor.NONE) {
            isIntakeEnabled = false;

            intake.reverse();
            try { Thread.sleep(400); } catch (InterruptedException e) {}
            intake.off();
            indexer.off();

        } else indexer.index(1);
    }

    public void shootSequence() {
        if (shooter.targetPower == Shooter.COAST_POWER) return;

        if (intake.on) {
            isIntakeEnabled = false;
            intake.off();
        }

        if (!indexer.RAPID_FIRE) indexer.indexPattern();

        while (!shooter.ready() && this.opMode.opModeIsActive()) { shooter.update(); try {Thread.sleep(20);} catch (InterruptedException e) {}}
        indexer.shoot(3);
        while (indexer.isBusy() && this.opMode.opModeIsActive()) { shooter.update(); try {Thread.sleep(20);} catch (InterruptedException e) {}}


        shooter.off();
        isIntakeEnabled = true;
        indexer.previousLastElement = Enums.ArtifactColor.NONE;

        indexer.home();
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
