package org.firstinspires.ftc.teamcode.Hardware.Util.SensorsEx;

import static org.firstinspires.ftc.teamcode.Hardware.Constants.DriveConstants.RANGE;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class AbsoluteAnalogEncoder {
    private final AnalogInput encoder;
    private double offset;
    private boolean inverted;

    private double pastPosition = 1;


    public AbsoluteAnalogEncoder(AnalogInput enc){
        encoder = enc;
        offset = 0;
        inverted = false;
    }

    public AbsoluteAnalogEncoder zero(double off){
        offset = off;
        return this;
    }

    public AbsoluteAnalogEncoder setInverted(boolean invert){
        inverted = invert;
        return this;
    }



    public AnalogInput getEncoder() {
        return encoder;
    }



    public boolean getDirection() {
        return inverted;
    }

    public double getCurrentPosition(AngleUnit unit) {
        // in range [-π, +π]
        double pos = AngleUnit.normalizeRadians(((!inverted
                ? 1 - getVoltage() / RANGE
                : getVoltage() / RANGE)
                * Math.PI * 2) - offset);

        double delta = AngleUnit.normalizeRadians(pos - pastPosition);

        // checks for crazy values when the encoder is close to zero
        if (Math.abs(delta) > 0.1 || Math.abs(delta) < 1) {
            pastPosition = pos;
        }

        return (unit == AngleUnit.DEGREES)
                ? Math.toDegrees(pastPosition)
                : pastPosition;
    }

    public double getVoltage(){
        return encoder.getVoltage();
    }
}