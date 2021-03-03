package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.misc.CommandSender
import org.firstinspires.ftc.teamcode.misc.motorAccelerationLimiter
import org.firstinspires.ftc.teamcode.robot.Conveyor.ConveyorConfig.conveyorPower
import org.firstinspires.ftc.teamcode.robot.WoENHardware.conveyorMotor
import org.firstinspires.ftc.teamcode.robot.WoENHardware.ringDetector
import org.firstinspires.ftc.teamcode.superclasses.Conveyor
import org.firstinspires.ftc.teamcode.superclasses.MultithreadRobotModule
import kotlin.math.abs

class Conveyor : MultithreadRobotModule(),
    Conveyor {
    private lateinit var conveyor: DcMotorEx
    private lateinit var sensorDistance: DistanceSensor
    private val distanceQueryTimeout = 300.0
    private val motorCurrentQueryTimeout = 100.0

    private val conveyorPowerSender = CommandSender {conveyor.power = it}

    private val conveyorAccelerationLimiter = motorAccelerationLimiter({conveyorPowerSender.send(it)}, 6.0)

    private val motorCurrentTimer = ElapsedTime()
    private val stackDetectionTimer = ElapsedTime()
    private var lastKnownDistance = 12.0
    private val distanceQueryTimer = ElapsedTime()
    private var lastKnownMotorCurrent = 0.0
    override var forceReverse = false
    private var currentMotorPower = 0.0
    override var enableConveyor = false
    private val motorCurrentQueryTimer = ElapsedTime()
    override var enableFullStackStopping = true
    override var reverseBeforeStop = true
    private var requestedPower = 0.0

    @Config
    internal object ConveyorConfig {
        @JvmField
        var conveyorPower = 1.0
        @JvmField
        var motorLockingCurrentTimeout = 800.0
        @JvmField
        var motorLockingReverseTime = 600.0
        @JvmField
        var stackDetectionTimeout = 1000.0
        @JvmField
        var stackDetectionReverseTime = 600.0
        @JvmField
        var distanceThreshold = 5.46
        @JvmField
        var currentThreshold = 2.5
    }

    override fun initialize() {
        initializecolor()
        initializedrive()
    }

    private fun initializecolor() {
        sensorDistance = ringDetector
    }

    private fun initializedrive() {
        conveyor = conveyorMotor
        conveyor.direction = DcMotorSimple.Direction.REVERSE //Should be intaking rings at +1.0
        conveyor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }


    private fun getdistance(): Double {
        if (distanceQueryTimer.milliseconds() > distanceQueryTimeout) {
            lastKnownDistance = sensorDistance.getDistance(DistanceUnit.CM)
            distanceQueryTimer.reset()
        }
        return lastKnownDistance
    }

    private val aMPS: Double
        get() {
            if (motorCurrentQueryTimer.milliseconds() > motorCurrentQueryTimeout) {
                lastKnownMotorCurrent = abs(conveyor.getCurrent(CurrentUnit.AMPS))
                motorCurrentQueryTimer.reset()
            }
            return lastKnownMotorCurrent
        }

    override fun updateControlHub() {
        if (!forceReverse && !(reverseBeforeStop && requestedPower == 0.0) && enableFullStackStopping)
            if (getdistance() >= ConveyorConfig.distanceThreshold) {
            stackDetectionTimer.reset() //Full collector detection
        }
    }

    val collectorIsFull : Boolean
        get() = stackDetectionTimer.milliseconds() > ConveyorConfig.stackDetectionTimeout && !forceReverse && enableFullStackStopping && requestedPower != 0.0
    val motorIsLocked: Boolean
        get() = motorCurrentTimer.milliseconds() > ConveyorConfig.motorLockingCurrentTimeout && !forceReverse && stackDetectionTimer.milliseconds() < ConveyorConfig.stackDetectionTimeout && requestedPower != 0.0

    override fun updateExpansionHub() {
        if (!forceReverse) {
            if ((stackDetectionTimer.milliseconds() > ConveyorConfig.stackDetectionTimeout || reverseBeforeStop && requestedPower == 0.0) && enableFullStackStopping) { //reverse+stop in case of ring detection
                motorCurrentTimer.reset()
                currentMotorPower =
                    if (stackDetectionTimer.milliseconds() < ConveyorConfig.stackDetectionTimeout + ConveyorConfig.stackDetectionReverseTime && reverseBeforeStop) -1.0 else 0.0
            } else if (motorCurrentTimer.milliseconds() > ConveyorConfig.motorLockingCurrentTimeout) //reverse after locking
            {
                if (motorCurrentTimer.milliseconds() < ConveyorConfig.motorLockingReverseTime + ConveyorConfig.motorLockingCurrentTimeout) {
                    currentMotorPower = - if(enableConveyor) conveyorPower else 0.0
                } else motorCurrentTimer.reset()
            } else {
                currentMotorPower = + if(enableConveyor) conveyorPower else 0.0
                if (aMPS < ConveyorConfig.currentThreshold) //locking detection
                    motorCurrentTimer.reset()
            }
        } else {
            currentMotorPower = -1.0
            motorCurrentTimer.reset()
        }
        conveyorAccelerationLimiter.setVelocity(currentMotorPower)
    }

    override fun updateOther() {
    }
}