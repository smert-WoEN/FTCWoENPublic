package org.firstinspires.ftc.teamcode.math

import org.firstinspires.ftc.teamcode.math.MathUtil.angleWrap
import org.firstinspires.ftc.teamcode.math.MathUtil.approxEquals
import java.util.*

open class Pose2D @JvmOverloads constructor(x: Double = 0.0, y: Double = 0.0, heading: Double = 0.0) : Vector2D(x, y), Cloneable {
    @JvmField var heading: Double = angleWrap(heading)

    constructor(p: Vector2D, heading: Double) : this(p.x, p.y, angleWrap(heading))

    operator fun plus(p2: Pose2D) = Pose2D(x + p2.x, y + p2.y, heading + p2.heading)

    operator fun times(p2: Pose2D) = Pose2D(x * p2.x, y * p2.y, heading * p2.heading)

    operator fun div(p2: Pose2D) = Pose2D(x / p2.x, y / p2.y, heading / p2.heading)

    operator fun div(d: Double) = Pose2D(x / d, y / d, heading / d)

    operator fun minus(p2: Pose2D) = Pose2D(x - p2.x, y - p2.y, heading - p2.heading)

    override operator fun times(d: Double) = Pose2D(x * d, y * d, heading * d)

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (other !is Pose2D) return false
        if (!super.equals(other as Vector2D?)) return false
        return approxEquals(other.heading, heading)
    }

    override fun hashCode() = x.hashCode() xor y.hashCode() xor heading.hashCode()

    override fun toString() = String.format(Locale.getDefault(), "{x: %.3f, y: %.3f, θ: %.3f}", x, y, heading)

    override fun clone() = Pose2D(x, y, heading)

}