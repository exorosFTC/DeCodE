package org.firstinspires.ftc.teamcode.Hardware.Util.SensorsEx;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.Hardware.Constants.Enums;
import org.firstinspires.ftc.teamcode.Hardware.Robot.Hardware;

public class ColorSensorEx {
    private final Hardware hardware;
    private final String name;
    private final LinearOpMode opMode;

    public static double MIN_SATURATION = 0.2; //saturation to consider a color
    public static double MIN_VALUE = 0.2; //minimum value brightness to detect presence
    public static final double GREEN_HUE_MIN = 85;
    public static final double GREEN_HUE_MAX = 170;
    public static double PURPLE_HUE_MIN = 270;
    public static double PURPLE_HUE_MAX = 290;

    public ColorSensorEx(LinearOpMode opMode, String name) {
        this.hardware = Hardware.getInstance(opMode);
        this.name = name;
        this.opMode = opMode;
    }



    public float[] getRGB() {
        NormalizedRGBA colors = hardware.color.get(name).getNormalizedColors();
        return new float[]{colors.red, colors.green, colors.blue};
    }
    //converts rgb to hsv, just returns float
    public float[] getHSV() {
        float[] rgb = getRGB();
        float r = rgb[0];
        float g = rgb[1];
        float b = rgb[2];

        float max = Math.max(r, Math.max(g, b));
        float min = Math.min(r, Math.min(g, b));
        float delta = max - min;

        float hue = 0;
        if (delta != 0) {
            if (max == r) {
                hue = 60 * (((g - b) / delta) % 6);
            } else if (max == g) {
                hue = 60 * (((b - r) / delta) + 2);
            } else {
                hue = 60 * (((r - g) / delta) + 4);
            }
            if (hue < 0) hue += 360;
        }

        float saturation = (max == 0) ? 0 : delta / max;
        float value = max;

        return new float[]{hue, saturation, value};
    }

    public double getAlpha() {
        return hardware.color.get(name).getNormalizedColors().alpha;
    }


    //detects if a ball is present and tells color
    public Enums.ArtifactColor getArtifactColor() {
        float[] hsv = getHSV();
        float hue = hsv[0];
        float saturation = hsv[1];
        float value = hsv[2];

        if (saturation < MIN_SATURATION || value < MIN_VALUE) {
            return Enums.ArtifactColor.NONE; //not enough color or brightness to detect
        }

        if (hue >= GREEN_HUE_MIN && hue <= GREEN_HUE_MAX) {
            return Enums.ArtifactColor.GREEN;
        } else if (hue >= PURPLE_HUE_MIN && hue <= PURPLE_HUE_MAX) {
            return Enums.ArtifactColor.PURPLE;
        }

        return Enums.ArtifactColor.NONE;
    }

    public boolean hasArtifact() {
        return getArtifactColor() != Enums.ArtifactColor.NONE || getAlpha() > 0.5; //alpha threshold for presence
    }
}