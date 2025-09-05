package org.firstinspires.ftc.teamcode.Hardware.Util.SensorsEx;

public class ColorEx {
    private double accuracyTreshold = 6;
    private double r, g, b;

    public ColorEx(double r, double g, double b, double accuracyThreshold) {
        this.r = r;
        this.g = g;
        this.b = b;
        this.accuracyTreshold = accuracyThreshold;
    }

    public ColorEx(double r, double g, double b) {
        this.r = r;
        this.g = g;
        this.b = b;
    }

    public boolean sameAs(ColorEx color) {
        return (Math.abs(color.getR() - r) <= accuracyTreshold &&
                Math.abs(color.getG() - g) <= accuracyTreshold &&
                Math.abs(color.getB() - b) <= accuracyTreshold);
    }

    public double getR() { return r; }

    public double getG() { return g;}

    public double getB() { return b; }

    public double getAccuracyThreshold() { return accuracyTreshold; }

    public void setAccuracyThreshold(double newAccuracyThreshold) { accuracyTreshold = newAccuracyThreshold; }
}
