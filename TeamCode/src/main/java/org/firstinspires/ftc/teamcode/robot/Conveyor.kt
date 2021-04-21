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
import org.firstinspires.ftc.teamcode.misc.MotorAccelerationLimiter
import org.firstinspires.ftc.teamcode.robot.Conveyor.ConveyorConfig.ratedConveyorPower
import org.firstinspires.ftc.teamcode.robot.WoENHardware.conveyorMotor
import org.firstinspires.ftc.teamcode.robot.WoENHardware.ringDetector
import org.firstinspires.ftc.teamcode.superclasses.Conveyor
import org.firstinspires.ftc.teamcode.superclasses.MultithreadedRobotModule
import kotlin.math.abs

class Conveyor : MultithreadedRobotModule(), Conveyor {
    private lateinit var conveyor: DcMotorEx
    private lateinit var sensorDistance: DistanceSensor
    private val distanceQueryTimeout = 300.0
    private val motorCurrentQueryTimeout = 100.0

    private val conveyorPowerSender = CommandSender({ conveyor.power = it })

    private val conveyorAccelerationLimiter = MotorAccelerationLimiter({ conveyorPowerSender.send(it) }, 6.0) //motorAccelerationLimiter({conveyorPowerSender.send(it)}, 6.0)

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

    @Config
    internal object ConveyorConfig {
        @JvmField var ratedConveyorPower = 1.0
        @JvmField var motorLockingCurrentTimeout = 800.0
        @JvmField var motorLockingReverseTime = 600.0
        @JvmField var stackDetectionTimeout = 1000.0
        @JvmField var stackDetectionReverseTime = 600.0
        @JvmField var distanceThreshold = 5.46
        @JvmField var currentThreshold = 4.0
    }

    override fun initialize() {
        initializeColor()
        initializeDrive()
    }

    private fun initializeColor() {
        sensorDistance = ringDetector
    }

    private fun initializeDrive() {
        conveyor = conveyorMotor
        conveyor.direction = DcMotorSimple.Direction.REVERSE //Should be intaking rings at +1.0
        conveyor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }


    private fun getDistance(): Double {
        if (distanceQueryTimer.milliseconds() > distanceQueryTimeout) {
            lastKnownDistance = sensorDistance.getDistance(DistanceUnit.CM)
            distanceQueryTimer.reset()
        }
        return lastKnownDistance
    }

    private val conveyorMotorCurrent: Double
        get() {
            if (!enableConveyor && !forceReverse) return 0.0
            if (motorCurrentQueryTimer.milliseconds() > motorCurrentQueryTimeout) {
                lastKnownMotorCurrent = abs(conveyor.getCurrent(CurrentUnit.AMPS))
                motorCurrentQueryTimer.reset()
            }
            return lastKnownMotorCurrent
        }

    override fun updateControlHub() {
        if (!forceReverse && !(reverseBeforeStop && !enableConveyor) && enableFullStackStopping) if (getDistance() >= ConveyorConfig.distanceThreshold) {
            stackDetectionTimer.reset() //Full collector detection
        }
    }

    val collectorIsFull: Boolean
        get() = stackDetectionTimer.milliseconds() > ConveyorConfig.stackDetectionTimeout && !forceReverse && enableFullStackStopping && enableConveyor
    val motorIsLocked: Boolean
        get() = motorCurrentTimer.milliseconds() > ConveyorConfig.motorLockingCurrentTimeout && !forceReverse && stackDetectionTimer.milliseconds() < ConveyorConfig.stackDetectionTimeout && enableConveyor

    override fun updateExpansionHub() {
        if (!forceReverse) {
            if ((stackDetectionTimer.milliseconds() > ConveyorConfig.stackDetectionTimeout || reverseBeforeStop && !enableConveyor) && enableFullStackStopping) { //reverse+stop in case of ring detection
                motorCurrentTimer.reset()
                currentMotorPower = if (stackDetectionTimer.milliseconds() < ConveyorConfig.stackDetectionTimeout + ConveyorConfig.stackDetectionReverseTime && reverseBeforeStop) -1.0 else 0.0
            } else if (motorCurrentTimer.milliseconds() > ConveyorConfig.motorLockingCurrentTimeout) //reverse after locking
            {
                if (motorCurrentTimer.milliseconds() < ConveyorConfig.motorLockingReverseTime + ConveyorConfig.motorLockingCurrentTimeout) {
                    currentMotorPower = -if (enableConveyor) ratedConveyorPower else 0.0
                } else motorCurrentTimer.reset()
            } else {
                currentMotorPower = +if (enableConveyor) ratedConveyorPower else 0.0
                if (conveyorMotorCurrent < ConveyorConfig.currentThreshold) //locking detection
                    motorCurrentTimer.reset()
            }
        } else {
            currentMotorPower = -ratedConveyorPower
            motorCurrentTimer.reset()
        }
        conveyorAccelerationLimiter.setVelocity(currentMotorPower)
    }

    override fun updateOther() {
    }
}