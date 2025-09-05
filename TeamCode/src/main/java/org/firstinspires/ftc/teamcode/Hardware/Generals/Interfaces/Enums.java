package org.firstinspires.ftc.teamcode.Hardware.Generals.Interfaces;

import static org.firstinspires.ftc.teamcode.Hardware.Generals.Constants.SystemConstants.outtakeMAX;

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


    enum Color{
        RED,
        BLUE,
        YELLOW,
        NONE
    }


    interface Pathing{
        enum Polynomial{
            UNDEFINED, constant, linear, quadratic, cubic, quartic, quintic, MULTIPLE
        }
    }

    interface IntakeEnums {
        enum IntakeAction{
            COLLECT,
            TRANSFER,

            BACK_UP,

            MOTOR_COLLECT,
            MOTOR_SPIT,
            MOTOR_STOP
        }

        enum IntakePosition{
            ZERO(0),
            IN_TRANSFER(0);

            public final int ticks;

            IntakePosition(int ticks) { this.ticks = ticks; }
        }
    }

    interface OuttakeEnums {
        enum LiftAction{
            ZERO(0),
            HANG(100),
            TRANSFER(0),
            MID_SMT(100),
            FULL(outtakeMAX),

            HIGH_BASKET(750),
            HIGH_RUNG(80),
            LOW_BASKET(300), // for when we're too op
            LOW_RUNG(0); // we don't do that here

            public final int ticks;

            LiftAction(int ticks) {
                this.ticks = ticks;
            }
        }

        enum OuttakeAction{
            PRE_TRANSFER,
            COLLECT_SPECIMENS,
            SCORE_SPECIMENS,
            SCORE_SAMPLES
        }
    }
}
