package org.firstinspires.ftc.teamcode.Hardware.Robot.Drivetrain.Swerve;

public class SwerveModuleState {
    private double moduleVelocity;
    private double moduleAngle;

    private double moduleAngularVelocity;
    private double moduleAcceleration;
    private double moduleAngularAcceleration;



    public SwerveModuleState(double vel, double angle) {
        this.moduleVelocity = vel;
        this.moduleAngle = angle;
    }

    public SwerveModuleState(double vel, double angle, double accel, double angAccel) {
        this.moduleVelocity = vel;
        this.moduleAngle = angle;
        this.moduleAcceleration = accel;
        this.moduleAngularAcceleration = angAccel;
    }



    public void normalizeModuleVelocity(double norm) { moduleVelocity /= norm; }



    public void setModuleVelocity(double val) { moduleVelocity = val; }

    public void setModuleAngle(double val) { moduleAngle = val; }

    public void setModuleAcceleration(double val) { moduleAcceleration = val; }

    public void setModuleAngularVelocity(double val) { moduleAngularVelocity = val; }

    public void setModuleAngularAcceleration(double val) { moduleAngularAcceleration = val; }



    public double getModuleVelocity() { return moduleVelocity; }

    public double getModuleAngle() { return moduleAngle; }

    public double getModuleAcceleration() { return moduleAcceleration; }

    public double getModuleAngularVelocity() { return moduleAngularVelocity; }

    public double getModuleAngularAcceleration() { return moduleAngularAcceleration; }
}
