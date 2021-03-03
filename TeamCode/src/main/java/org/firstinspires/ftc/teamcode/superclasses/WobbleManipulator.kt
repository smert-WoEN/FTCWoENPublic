package org.firstinspires.ftc.teamcode.superclasses

interface WobbleManipulator {
    enum class Position {
        UP, DOWN, MEDIUM
    }

    fun setAngle(Positions: Position)
    fun grabWobble(dograb: Boolean)
    fun upmediumdown(upmedium: Boolean, updown: Boolean)
}