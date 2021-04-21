package org.firstinspires.ftc.teamcode.misc

import com.qualcomm.robotcore.util.ElapsedTime
import java.util.function.DoubleConsumer
import java.util.function.DoubleSupplier
import kotlin.math.abs
import kotlin.math.sign

class RegulatorPIDVAS (private val doubleConsumer: DoubleConsumer, private val doubleVelocity: DoubleSupplier, private val doubleSupplier: DoubleSupplier, private val kP: DoubleSupplier, private val kI: DoubleSupplier,
                       private val kD: DoubleSupplier, private val kV: DoubleSupplier, private val kA: DoubleSupplier, private val kS: DoubleSupplier, private val maxI: DoubleSupplier, private val kV_referenceVoltage: DoubleSupplier, private val activeBraking: Boolean = true){
    private val updateTime = ElapsedTime()
    private var velocityError = 0.0
    private var velocityErrorOld = 0.0
    private var P = 0.0
    private var I = 0.0
    private var D = 0.0
    private var V = 0.0
    private var A = 0.0
    private var S = 0.0
    private var power = 0.0
    private var timeOld = 0.0
    private var timeDelta = 0.0
    private var voltageDelta = 0.0
    private var velocityTargetOld = 0.0
    private val maxInt16 = 32767.0
    private var currentVelocity = 0.0
    /*fun updateCoefficients() {

    }*/
    fun update(target: Double):Double {
        timeDelta = updateTime.seconds() - timeOld
        timeOld = updateTime.seconds()
        currentVelocity = doubleVelocity.asDouble
        if (target != 0.0 || activeBraking) {
            voltageDelta = kV_referenceVoltage.asDouble / doubleSupplier.asDouble
            velocityError = target - currentVelocity
            P = velocityError * kP.asDouble
            D = (velocityError - velocityErrorOld) * kD.asDouble / timeDelta
            I += (kI.asDouble * velocityError) * timeDelta
            if(target == 0.0) I = .0
            if (abs(I) > maxI.asDouble) I = sign(I) * maxI.asDouble
            V = kV.asDouble * target * voltageDelta
            A = kA.asDouble * (target - velocityTargetOld) / timeDelta * voltageDelta
            S = kS.asDouble * sign(target) * voltageDelta
            power = (P + I + D + V + A + S) / maxInt16
            velocityErrorOld = velocityError
            velocityTargetOld = target
            if (!activeBraking && sign(target) != sign(power)) power = 0.0
        } else {
            voltageDelta = 0.0
            velocityError = 0.0
            power = 0.0
            velocityErrorOld = 0.0
            I = 0.0
            velocityTargetOld = 0.0
        }
        doubleConsumer.accept(power)
        return power
    }
}