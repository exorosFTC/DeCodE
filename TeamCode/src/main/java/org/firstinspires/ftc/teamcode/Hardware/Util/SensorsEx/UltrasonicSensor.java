package org.firstinspires.ftc.teamcode.Hardware.Util.SensorsEx;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;


public class UltrasonicSensor {
    private final AnalogInput sensor;
    private final LinearOpMode opMode;

    private final double MAX_mm = 5000;
    private final double MAX_voltage = 3.3;
    private final double multiplier = MAX_mm / MAX_voltage;;


    // Kalman filter variables
    private double kFilteredDistance = -1;
    private double P = 1;
    public double Q = 300;
    public double R = 0.000001;




    public UltrasonicSensor(LinearOpMode opMode, String input) {
        sensor = opMode.hardwareMap.get(AnalogInput.class, input);
        this.opMode = opMode;
    }

    public UltrasonicSensor(LinearOpMode opMode, AnalogInput input) {
        sensor = input;
        this.opMode = opMode;
    }

    public double getDistance(DistanceUnit unit) {
        switch (unit) {
            case MM: { return sensor.getVoltage() * multiplier; }
            case CM: { return sensor.getVoltage() * multiplier * 0.1; }
            case INCH: { return sensor.getVoltage() * multiplier / 25.4; }
        }
        return sensor.getVoltage();
    }

    public double getVoltage() { return sensor.getVoltage(); }


    /**
     * Returns a filtered distance measurement using a Kalman filter.
     * The filter operates in millimeters and then converts the value to the requested unit.
     */
    public double getFilteredDistance(DistanceUnit unit) {
        // Get raw measurement in mm.
        double measurement = sensor.getVoltage() / MAX_voltage * MAX_mm;

        // If this is the first reading, initialize the filter state.
        if (kFilteredDistance < 0) {
            kFilteredDistance = measurement;
        }

        // --- Kalman Filter Update ---
        // 1. Prediction update: Increase error covariance.
        P = P + Q;

        // 2. Measurement update: Calculate Kalman Gain.
        double K = P / (P + R);

        // 3. Update the estimate with the measurement.
        kFilteredDistance = kFilteredDistance + K * (measurement - kFilteredDistance);

        // 4. Update the error covariance.
        P = (1 - K) * P;
        // --------------------------------

        // Convert the filtered value to the requested unit.
        switch (unit) {
            case MM:   return kFilteredDistance;
            case CM:   return kFilteredDistance / 10.0;
            case INCH: return kFilteredDistance / 25.4;
        }

        return 0;
    }

}
