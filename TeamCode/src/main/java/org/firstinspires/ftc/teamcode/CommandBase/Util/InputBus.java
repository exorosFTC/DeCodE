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
    public final AtomicBoolean evShootSorted    = new AtomicBoolean(false);
    public final AtomicBoolean evShootUnsorted  = new AtomicBoolean(false);
    public final AtomicBoolean evHomeIndexer    = new AtomicBoolean(false);
    public final AtomicBoolean evLockX          = new AtomicBoolean(false);
    public final AtomicBoolean evStartLift      = new AtomicBoolean(false);
    public final AtomicBoolean evResetHeading   = new AtomicBoolean(false);
}