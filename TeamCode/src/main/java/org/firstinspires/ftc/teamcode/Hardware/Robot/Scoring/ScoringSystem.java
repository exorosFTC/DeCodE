package org.firstinspires.ftc.teamcode.Hardware.Robot.Scoring;

import static org.firstinspires.ftc.teamcode.Hardware.Constants.HardwareNames.BreakBeam;
import static org.firstinspires.ftc.teamcode.Hardware.Constants.HardwareNames.IntakeColor;
import static org.firstinspires.ftc.teamcode.Hardware.Robot.Scoring.Subsystems.Indexer.elements;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Constants.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Scoring.Subsystems.Indexer;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Scoring.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Scoring.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Scoring.Subsystems.Tilt;
import org.firstinspires.ftc.teamcode.Hardware.Util.SensorsEx.ColorSensorEx;

public class ScoringSystem {
    private final Hardware hardware;
    private final LinearOpMode opMode;

    private final ElapsedTime timer;
    private final ColorSensorEx color;

    public Intake intake;
    public Indexer indexer;
    public Shooter shooter;
    public Tilt tilt;

    private boolean isIntakeEnabled = true;
    private boolean isShooterEnabled = false;

    public ScoringSystem(LinearOpMode opMode) {
        this.hardware = Hardware.getInstance(opMode);
        this.opMode = opMode;

        timer = new ElapsedTime();
        color = new ColorSensorEx(opMode, IntakeColor);

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
        boolean beam = hardware.digital.get(BreakBeam).getState();
        if (!beam) return;

        timer.reset();
        while (opMode.opModeIsActive() && timer.seconds() < 1.6 && !color.hasArtifact()) {}

        if (color.hasArtifact()) { elements.set(0, color.getArtifactColor());}
        else { elements.set(0, Enums.ArtifactColor.PURPLE); } //for now just pretend it's purple

        if (elements.get(2) != Enums.ArtifactColor.NONE) {
            isIntakeEnabled = false;
            isShooterEnabled = true;

            intake.off();
        } else { indexer.index(1); }
    }
}
