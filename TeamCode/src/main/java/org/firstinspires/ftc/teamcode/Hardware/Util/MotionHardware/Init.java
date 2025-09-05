package org.firstinspires.ftc.teamcode.Hardware.Util.MotionHardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public interface Init {

    static DcMotorEx initializeMotor(DcMotorEx motor) {
        MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1);
        motor.setMotorType(motorConfigurationType);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        return motor;
    }



    static DcMotorEx initializeMotor(DcMotorEx motor, DcMotor.RunMode runMode) {
        MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1);
        motor.setMotorType(motorConfigurationType);

        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(runMode);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        return motor;
    }

    static DcMotorEx initializeMotor(DcMotorEx motor, DcMotorSimple.Direction direction) {
        MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1);
        motor.setMotorType(motorConfigurationType);

        motor.setDirection(direction);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        return motor;
    }




    static DcMotorEx initializeMotor(DcMotorEx motor, DcMotor.RunMode runMode, DcMotorSimple.Direction direction) {
        MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1);
        motor.setMotorType(motorConfigurationType);

        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(runMode);

        motor.setDirection(direction);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        return motor;
    }



    static void resetEncoders(DcMotorEx motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(0);
    }

    static void resetEncoders(DcMotorEx motor, DcMotor.RunMode runMode) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(runMode);
    }

    static double getCurrent(DcMotorEx motor) { return motor.getCurrent(CurrentUnit.AMPS); }

    static boolean isConstrained(DcMotorEx motor, double currentThreshold) { return getCurrent(motor) > currentThreshold; }

    static boolean isConstrained(double amperage, double currentThreshold) { return amperage > currentThreshold; }
}
