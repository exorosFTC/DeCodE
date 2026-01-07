package org.firstinspires.ftc.teamcode.CommandBase.Constants;

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

    enum Pipeline{
        RANDOMIZATION,
        BLUE_GOAL,
        RED_GOAL
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

    enum HeadingMode {
        FACE_GOAL,
        FIXED,
        LERP
    }
}
