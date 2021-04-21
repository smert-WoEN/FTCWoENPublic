package org.firstinspires.ftc.teamcode.misc

import com.qualcomm.robotcore.util.ElapsedTime
import java.util.function.DoubleConsumer
import java.util.function.DoubleSupplier
import kotlin.math.abs
import kotlin.math.sign

class MotorAccelerationLimiter constructor(private val motorToControl: DoubleConsumer, private val maxAcceleration: DoubleSupplier) {

    constructor(motorToControl: DoubleConsumer,maxAcceleration: Double) : this(motorToControl, DoubleSupplier {maxAcceleration})

    private val loopTime = ElapsedTime()
    private var currentVelocity = 0.0
    fun setVelocity(requestedVelocity: Double) {
        currentVelocity += abs(requestedVelocity - currentVelocity).coerceAtMost(abs(loopTime.seconds() * maxAcceleration.asDouble)) * sign(requestedVelocity - currentVelocity)
        if (requestedVelocity == 0.0) motorToControl.accept(0.0) else motorToControl.accept(currentVelocity)
        loopTime.reset()
    }

    init {
        loopTime.reset()
    }
}