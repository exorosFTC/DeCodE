package org.firstinspires.ftc.teamcode.Hardware.Constants;

public interface Enums {

    enum Hubs{
        CONTROL_HUB,
        EXPANSION_HUB,
        ALL
    }

    enum OpMode{
        TELE_OP,
        AUTONOMUS
    }

    enum Gamepads{
        G1, G2,
        BOTH,
        NONE
    }

    enum Pipelines{
        DETECTING_PROP,
    }

    enum Randomization{
        LEFT,
        CENTER,
        RIGHT
    }




    //game specific
    enum ArtifactColor{
        PURPLE,
        GREEN,
        NONE,
    }

    enum IndexerPos{
        COLLECT_1(0),
        COLLECT_2(120),
        COLLECT_3(240);

        private final int deg;

        IndexerPos(int deg) {
            this.deg = deg;
        }

        public int getDeg() {
            return deg;
        }
    }
}
