package org.firstinspires.ftc.teamcode.math

import org.firstinspires.ftc.teamcode.math.MathUtil.approxEquals
import org.firstinspires.ftc.teamcode.math.MathUtil.cosFromSin
import java.util.*
import kotlin.math.atan2
import kotlin.math.sin
import kotlin.math.sqrt

open class Vector2D constructor(var x: Double = 0.0, var y: Double = 0.0): Cloneable {
    fun rotated(angle: Double): Vector2D {
        val sinA = sin(angle)
        val cosA = cosFromSin(sinA, angle)
        return Vector2D(x * cosA - y * sinA, x * sinA + y * cosA)
    }

    open fun rotatedCW(angle: Double): Vector2D {
        val sinA = sin(angle)
        val cosA = cosFromSin(sinA, angle)
        return Vector2D(x * cosA + y * sinA, -x * sinA + y * cosA)
    }

    open fun normalize(): Vector2D {
        val r = radius()
        return if (r != 0.0) {
            Vector2D(x / r, y / r)
        } else this
    }

    operator fun plus(p: Vector2D): Vector2D {
        return Vector2D(x + p.x, y + p.y)
    }

    operator fun minus(p: Vector2D): Vector2D {
        return Vector2D(x - p.x, y - p.y)
    }

    fun aTan() = atan2(y, x)

    fun aCot() = atan2(x, y)

    open fun radius() = sqrt(x * x + y * y)

    open operator fun times(d: Double) = Vector2D(x * d, y * d)

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (other !is Vector2D) return false
        return approxEquals(other.x, x) && approxEquals(other.y, y)
    }

    override fun hashCode() = x.hashCode() xor y.hashCode()

    override fun toString() = String.format(Locale.getDefault(), "(%.1f, %.1f)", x, y)

    public override fun clone(): Vector2D = Vector2D(x, y)
}