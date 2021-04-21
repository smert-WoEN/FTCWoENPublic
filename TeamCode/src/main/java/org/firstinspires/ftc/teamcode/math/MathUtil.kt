package org.firstinspires.ftc.teamcode.math

import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.sqrt

object MathUtil {

    private const val EPSILON = 1e-6

    fun approxEquals(d1: Double, d2: Double, minDelta: Double = EPSILON) = abs(d1 - d2) < minDelta

    fun angleWrap(angle: Double): Double {
        var correctedAngle = angle
        while (correctedAngle > PI) correctedAngle -= PI * 2
        while (correctedAngle < -PI) correctedAngle += PI * 2
        return correctedAngle
    }

    fun angleAverage(angle1: Double, angle2: Double) = angleWrap(angle1 + angleWrap(angle2 - angle1) / 2.0)

    fun minAbs(a: Double, b: Double) = if (abs(a) < abs(b)) a else b

    fun cosFromSin(sin: Double, angle: Double): Double {
        val correctedAngle = angleWrap(angle)
        var cos = sqrt(1 - sin * sin)
        if (correctedAngle > PI / 2 || correctedAngle < -PI / 2) cos *= -1.0
        return cos
    }
}