package org.firstinspires.ftc.teamcode.CommandBase.Util;

import java.util.concurrent.atomic.AtomicBoolean;

public class InputBus {
    // continuous
    public volatile double lx, ly, rx;
    public volatile double lt, rt;
    public volatile boolean spinupShooter;
    public volatile boolean lockToGoal;

    public volatile double ly2;

    // one-shot events
    public final AtomicBoolean evToggleIntake   = new AtomicBoolean(false);
    public final AtomicBoolean evReverseIntake  = new AtomicBoolean(false);
    public final AtomicBoolean evSort           = new AtomicBoolean(false);
    public final AtomicBoolean evShoot          = new AtomicBoolean(false);
    public final AtomicBoolean evHomeIndexer    = new AtomicBoolean(false);
    public final AtomicBoolean evLockX          = new AtomicBoolean(false);
    public final AtomicBoolean evStartLift      = new AtomicBoolean(false);
    public final AtomicBoolean evResetHeading   = new AtomicBoolean(false);
    public final AtomicBoolean evResetPosition  = new AtomicBoolean(false);
    public final AtomicBoolean evSetBlue        = new AtomicBoolean(false);
    public final AtomicBoolean evSetRed         = new AtomicBoolean(false);
}