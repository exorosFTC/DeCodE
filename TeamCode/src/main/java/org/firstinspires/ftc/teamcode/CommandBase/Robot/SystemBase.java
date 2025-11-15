package org.firstinspires.ftc.teamcode.CommandBase.Robot;

public abstract class SystemBase {
    public boolean on = false;

    public abstract void read();
    public abstract void write();

    public void on() { this.on = true; }
    public void off() { this.on = false; }
}
