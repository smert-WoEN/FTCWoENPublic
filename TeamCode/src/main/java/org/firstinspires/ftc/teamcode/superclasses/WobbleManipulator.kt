package org.firstinspires.ftc.teamcode.superclasses

interface WobbleManipulator {
    enum class Position {
        UP, DOWN, MEDIUM
    }

    fun setAngle(Positions: Position)
    fun grabWobble(doGrab: Boolean)
    fun upMediumDown(upMedium: Boolean, upDown: Boolean)
}