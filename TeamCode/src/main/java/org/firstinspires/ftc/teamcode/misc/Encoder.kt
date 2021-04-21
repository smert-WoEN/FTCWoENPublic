package org.firstinspires.ftc.teamcode.misc

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.MovingStatistics
import kotlin.math.abs
import kotlin.math.sign


/**
 * Wraps a motor instance to provide corrected velocity counts and allow reversing without changing the corresponding
 * slot's motor direction
 */
class Encoder(private val motor: DcMotorEx, var direction: Direction) {


    enum class Direction(val multiplier: Int) {
        FORWARD(1), REVERSE(-1);
    }
    private val CPS_STEP = 0x10000
    private fun inverseOverflow(input: Double, estimate: Double): Double {
        var real = input
        while (abs(estimate - real) > CPS_STEP / 2.0) {
            real += sign(estimate - real) * CPS_STEP
        }
        return real
    }
    private var velocityEstimates = MovingStatistics(5)

    private val clock: ElapsedTime = ElapsedTime()
    private var lastPosition: Int
    private var lastUpdateTime: Double
    val currentPosition: Int
        get() {
            val multiplier = direction.multiplier * if (motor.direction == DcMotorSimple.Direction.FORWARD) 1 else -1
            val currentPosition = motor.currentPosition * multiplier
            if (currentPosition != lastPosition) {
                val currentTime = clock.seconds()
                val dt = currentTime - lastUpdateTime
                velocityEstimates.add((currentPosition - lastPosition) / dt)
                lastPosition = currentPosition
                lastUpdateTime = currentTime
            }
            return currentPosition
        }
    val rawVelocity: Double
        get() {
            val multiplier = direction.multiplier * if (motor.direction == DcMotorSimple.Direction.FORWARD) 1 else -1
            return motor.velocity * multiplier
        }
    val correctedVelocity: Double
        get() = inverseOverflow(rawVelocity, velocityEstimates.mean)

    init {
        lastPosition = 0
        velocityEstimates = MovingStatistics(5)
        lastUpdateTime = clock.seconds()
    }
}