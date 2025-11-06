package org.firstinspires.ftc.teamcode.Hardware.Robot.Scoring;

import static org.firstinspires.ftc.teamcode.Hardware.Constants.HardwareNames.BreakBeam;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.HardwareNames.IndexerMotor;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.HardwareNames.IntakeColor;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.HardwareNames.IntakeMotor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    public ElapsedTime timer;

    public boolean isIntakeEnabled = true;
    public boolean isShooterEnabled = false;

    public ScoringSystem(LinearOpMode opMode) {
        this.hardware = Hardware.getInstance(opMode);


        timer = new ElapsedTime();

        intake = new Intake(opMode);
        indexer = new Indexer(opMode);
        shooter = new Shooter(opMode);
        tilt = new Tilt(opMode);
    }



    public void update() {
        if (isIntakeEnabled) updateIntake();
        else if (isShooterEnabled) shooter.update();
    }

    public void updateIntake() {
        double distance = hardware.color.get(IntakeColor).getDistance(DistanceUnit.MM);

        hardware.telemetry.addData("ELEMENT 1", indexer.elements.get(0));
        hardware.telemetry.addData("ELEMENT 2", indexer.elements.get(1));
        hardware.telemetry.addData("ELEMENT 3", indexer.elements.get(2));
        hardware.telemetry.addData("distance", distance);
        hardware.telemetry.addData("AMPS", hardware.motors.get(IntakeMotor).getCurrent(CurrentUnit.AMPS));
        hardware.telemetry.addData("hasArtifact", distance < 78);
        hardware.telemetry.addData("indexer pos", hardware.motors.get(IndexerMotor).getCurrentPosition());

        if (hardware.motors.get(IntakeMotor).getCurrent(CurrentUnit.AMPS) > 8.5)
            new Thread(() -> {
                 intake.reverse();
                try { Thread.sleep(200); } catch (InterruptedException e) {}
                intake.on();
            }).start();

        if (distance > 78 && indexer.elements.get(0) != Enums.ArtifactColor.NONE) indexer.elements.set(0, Enums.ArtifactColor.NONE);
        if (distance > 78 || indexer.isBusy()) return;

        double g = hardware.color.get(IntakeColor).green(),
                b = hardware.color.get(IntakeColor).blue();

        if (g > b) indexer.elements.set(0, Enums.ArtifactColor.GREEN);
        else indexer.elements.set(0, Enums.ArtifactColor.PURPLE);



        if (indexer.elements.get(2) != Enums.ArtifactColor.NONE) {
            //isIntakeEnabled = false;
            //isShooterEnabled = true;

            intake.off();
        } else { indexer.index(1); }
    }
}
