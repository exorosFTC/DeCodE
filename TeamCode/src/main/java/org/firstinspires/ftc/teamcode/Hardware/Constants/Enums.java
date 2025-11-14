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

    enum HeadingMode {
        PATH_TANGENT,
        FACE_GOAL,
        FIXED
    }

    enum SwerveMode {
        ECHO,
        SPORT
    }
}
