package org.firstinspires.ftc.teamcode.math

import org.firstinspires.ftc.teamcode.math.MathUtil.approxEquals
import org.firstinspires.ftc.teamcode.math.MathUtil.minAbs
import java.util.*
import kotlin.math.sqrt
import kotlin.math.withSign

class Vector3D constructor(x: Double = 0.0, y: Double = 0.0, var z: Double = 0.0) : Vector2D(x, y), Cloneable {
    constructor(p: Pose2D) : this(p.x, p.y, p.heading)
    constructor(p: Vector2D?, z: Double) : this(p!!.x, p.y, z)

    operator fun plus(p2: Vector3D) = Vector3D(x + p2.x, y + p2.y, z + p2.z)

    operator fun times(p2: Vector3D) = Vector3D(x * p2.x, y * p2.y, z * p2.z)

    operator fun div(d: Double) = Vector3D(x / d, y / d, z / d)

    operator fun minus(p2: Vector3D) = Vector3D(x - p2.x, y - p2.y, z - p2.z)

    override fun times(d: Double) = Vector3D(x * d, y * d, z * d)

    override fun rotatedCW(angle: Double) = Vector3D(super.rotatedCW(angle), z)

    override fun normalize(): Vector3D {
        val r = radius()
        return if (r != 0.0) Vector3D(x / r, y / r, z / r) else this
    }

    override fun radius() = sqrt(x * x + y * y + z * z)

    fun clampAbs(p2: Vector3D) = Vector3D(minAbs(x, p2.x).withSign(x),minAbs(y, p2.y).withSign(y),minAbs(z, p2.z).withSign(z))

    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (other !is Vector3D) return false
        if (!super.equals(other)) return false
        return approxEquals(other.z, z)
    }

    override fun hashCode() = x.hashCode() xor y.hashCode() xor z.hashCode()

    override fun toString() = String.format(Locale.getDefault(), "{x: %.3f, y: %.3f, z: %.3f}", x, y, z)

    override fun clone() = Vector3D(x, y, z)
}