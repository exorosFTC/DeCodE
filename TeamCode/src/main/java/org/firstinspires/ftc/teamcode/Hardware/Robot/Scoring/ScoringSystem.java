package org.firstinspires.ftc.teamcode.Hardware.Robot.Scoring;

import static org.firstinspires.ftc.teamcode.Hardware.Constants.HardwareNames.IntakeColor;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.HardwareNames.IntakeMotor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.Constants.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Scoring.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Scoring.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Scoring.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Scoring.Subsystems.Tilt;

public class ScoringSystem {
    private final Hardware hardware;

    public Intake intake;
    public Indexer indexer;
    public Shooter shooter;
    public Tilt tilt;

    public boolean isIntakeEnabled = true;

    private double lastDistance = 1000;
    private int loopCount = 0;

    public ScoringSystem(LinearOpMode opMode) {
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
        double distance = hardware.color.get(IntakeColor).getDistance(DistanceUnit.MM);

        hardware.telemetry.addData("ELEMENT 1", indexer.elements.get(0));
        hardware.telemetry.addData("ELEMENT 2", indexer.elements.get(1));
        hardware.telemetry.addData("ELEMENT 3", indexer.elements.get(2));
        hardware.telemetry.addData("isIntakeEnabled", isIntakeEnabled);
        hardware.telemetry.addData("hasArtifact", distance < 78);
        hardware.telemetry.addData("isBusy indexer", indexer.isBusy());
        hardware.telemetry.addData("distance", distance);

        if (hardware.motors.get(IntakeMotor).getCurrent(CurrentUnit.AMPS) > 8.5)
            new Thread(() -> {
                 intake.reverse();
                try { Thread.sleep(200); } catch (InterruptedException e) {}
                intake.on();
            }).start();

        if (Math.abs(distance - lastDistance) < 2) loopCount += 1;
        else loopCount = 0;

        lastDistance = distance;


        if (indexer.elements.contains(Enums.ArtifactColor.NONE) && !isIntakeEnabled) isIntakeEnabled = true;

        if (distance > 78 && indexer.elements.get(0) != Enums.ArtifactColor.NONE) indexer.elements.set(0, Enums.ArtifactColor.NONE);
        if (distance > 78 || indexer.isBusy() || !isIntakeEnabled || loopCount < 3) return;



        double g = hardware.color.get(IntakeColor).green(),
                b = hardware.color.get(IntakeColor).blue();

        if (g > b) indexer.elements.set(0, Enums.ArtifactColor.GREEN);
        else indexer.elements.set(0, Enums.ArtifactColor.PURPLE);

        if (indexer.elements.get(2) != Enums.ArtifactColor.NONE) {
            isIntakeEnabled = false;

            intake.reverse();
            try { Thread.sleep(200); } catch (InterruptedException e) {}
            intake.off();
        } else indexer.index(1);
    }
}
