package org.firstinspires.ftc.teamcode.Hardware.Constants.Interfaces;

public interface Enums {

    enum Hubs{
        CONTROL_HUB,
        EXPANSION_HUB,
        ALL
    }

    enum Pipelines{
        DETECTING_PROP,
        DETECTING_SAMPLE
    }

    enum Randomization{
        LEFT,
        CENTER,
        RIGHT
    }

    enum OpMode{
        TELE_OP,
        AUTONOMUS
    }

    enum Access{
        INTAKE,
        OUTTAKE
    }

    enum Gamepads{
        G1, G2,
        BOTH,
        NONE
    }

    interface Pathing{
        enum Polynomial{
            UNDEFINED, constant, linear, quadratic, cubic, quartic, quintic, MULTIPLE
        }
    }
}
