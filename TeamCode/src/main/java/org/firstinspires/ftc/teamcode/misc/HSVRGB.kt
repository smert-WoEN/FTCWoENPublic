package org.firstinspires.ftc.teamcode.misc

import org.firstinspires.ftc.teamcode.math.Vector3D
import kotlin.math.floor

object HSVRGB {
    /**
     * @param H 0-360
     * @param S 0-100
     * @param V 0-100
     * @return color in hex string
     */
    fun convert(H: Float, S: Float, V: Float): Vector3D {
        val R: Float
        val G: Float
        val B: Float
        val normH = H / 360f
        val normS = S / 100f
        val normV = V / 100f
        if (normS == 0f) {
            R = normV * 255
            G = normV * 255
            B = normV * 255
        } else {
            var var_h = normH * 6
            if (var_h == 6f) var_h = 0f // H must be < 1
            val var_i = floor(var_h.toDouble()).toInt() // Or ... var_i =
            // floor( var_h )
            val var_1 = normV * (1 - normS)
            val var_2 = normV * (1 - normS * (var_h - var_i))
            val var_3 = normV * (1 - normS * (1 - (var_h - var_i)))
            val var_r: Float
            val var_g: Float
            val var_b: Float
            if (var_i == 0) {
                var_r = normV
                var_g = var_3
                var_b = var_1
            } else if (var_i == 1) {
                var_r = var_2
                var_g = normV
                var_b = var_1
            } else if (var_i == 2) {
                var_r = var_1
                var_g = normV
                var_b = var_3
            } else if (var_i == 3) {
                var_r = var_1
                var_g = var_2
                var_b = normV
            } else if (var_i == 4) {
                var_r = var_3
                var_g = var_1
                var_b = normV
            } else {
                var_r = normV
                var_g = var_1
                var_b = var_2
            }
            R = var_r * 255 // RGB results from 0 to 255
            G = var_g * 255
            B = var_b * 255
        }
        return Vector3D(R.toDouble(), G.toDouble(), B.toDouble())
    }
}