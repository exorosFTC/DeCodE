package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces.Enums;

public abstract class ExoMode extends LinearOpMode {
    protected abstract void Init();

    protected abstract void WhenStarted();

    protected abstract void InitializeThreads();

    protected abstract void Loop();

    protected void WaitForStart() { super.waitForStart(); }

    protected void AutonomusTasks() { idle(); }

    @Override
    public void runOpMode() throws InterruptedException {
        Init();
        while (opModeInInit()) WaitForStart();

        WhenStarted();
        try { while (opModeIsActive()) Loop(); } catch (RuntimeException e) {}
    }
}