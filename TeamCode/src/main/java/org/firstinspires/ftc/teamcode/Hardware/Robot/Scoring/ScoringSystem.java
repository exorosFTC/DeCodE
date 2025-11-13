package org.firstinspires.ftc.teamcode.Hardware.Robot.Scoring;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Constants.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Swerve.SwerveDrive;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Scoring.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Scoring.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Scoring.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Scoring.Subsystems.Tilt;
import org.firstinspires.ftc.teamcode.Hardware.Robot.SystemBase;

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

        if (Math.abs(colorDistance - lastDistance) < 2) loopCount += 1;
        else loopCount = 0;
        lastDistance = colorDistance;


        if (indexer.elements.contains(Enums.ArtifactColor.NONE) && !isIntakeEnabled) isIntakeEnabled = true;

        if (colorDistance > 78 && indexer.elements.get(0) != Enums.ArtifactColor.NONE) indexer.elements.set(0, Enums.ArtifactColor.NONE);
        if (colorDistance > 78 || indexer.isBusy() || !indexer.on || !isIntakeEnabled || loopCount < 2) return;

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

    public void shootSequence(SwerveDrive swerve) {
        if (shooter.targetPower == 0) return;

        swerve.setLockedX(true);
        hardware.telemetry.clear();

        if (intake.on) {
            isIntakeEnabled = false;
            intake.off();
        }

        while (!shooter.ready() && this.opMode.opModeIsActive()) { shooter.read(); shooter.update(); indexer.read(); shooter.write(); hardware.bulk.clearCache(Enums.Hubs.ALL); }
        indexer.shoot(3);
        while (indexer.isBusy() && this.opMode.opModeIsActive()) { shooter.read(); shooter.update(); indexer.read(); shooter.write(); hardware.bulk.clearCache(Enums.Hubs.ALL); }


        shooter.off();
        isIntakeEnabled = true;
        indexer.previousLastElement = Enums.ArtifactColor.NONE;

        swerve.setLockedX(false);

        indexer.home();
    }

    @Override
    public void read() {
        colorDistance = hardware.IntakeColor.getDistance(DistanceUnit.MM);
        colorValues = hardware.IntakeColor.getNormalizedColors();

        intake.read();
        indexer.read();
        shooter.read();
        tilt.read();
    }

    @Override
    public void write() {
        intake.write();
        indexer.write();
        shooter.write();
        tilt.write();
    }
}
