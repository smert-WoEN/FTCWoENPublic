package org.firstinspires.ftc.teamcode.robot.legacy

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.misc.CommandSender
import org.firstinspires.ftc.teamcode.robot.WoENHardware.conveyorMotor
import org.firstinspires.ftc.teamcode.robot.WoENHardware.ringDetector
import org.firstinspires.ftc.teamcode.superclasses.Conveyor
import org.firstinspires.ftc.teamcode.superclasses.MultithreadRobotModule

@Deprecated("")
class Conveyor() : MultithreadRobotModule(),
    Conveyor {
    private val conveyorTime = ElapsedTime()
    private val backOnTime = ElapsedTime()
    private val pauseTime = ElapsedTime()
    private val backOnAftertime = ElapsedTime()
    private lateinit var conveyorm: DcMotorEx
    override var enableConveyor = false
    set(value) {
        field = value
        conveyorPower = if (value) 1.0 else 0.0
    }

    private lateinit var sensorDistance: DistanceSensor
    private val conveyorPowerSender = CommandSender { p: Double -> conveyorm.power = -p }
    private var full = false
    private var backOn = false
    private var stop = false
    override var forceReverse = true
    override var enableFullStackStopping = true
    override var reverseBeforeStop = true
    private var timelock = 0.0
    private var conveyorPower = 0.0
    private var distance = 0.0
    private var current = 0.0
    override fun initialize() {
        initializecolor()
        initializedrive()
    }

    override fun start() {
        conveyorm.power = 0.0
        backOn = false
        stop = false
        conveyorPower = 0.0
    }

    private fun initializecolor() {
        sensorDistance = ringDetector
    }

    private fun initializedrive() {
        conveyorm = conveyorMotor
        conveyorm.direction = DcMotorSimple.Direction.FORWARD
        conveyorm.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    private fun getdistance(): Double {
        //return 10;
        return sensorDistance.getDistance(DistanceUnit.CM)
    }

    override fun updateControlHub() {
        if (pauseTime.milliseconds() >= 100) {
            pauseTime.reset()
            distance = if (!this.enableFullStackStopping) getdistance() else 10.0
        }
    }

    override fun updateExpansionHub() {
        current = conveyorm.getCurrent(CurrentUnit.AMPS)
        if (distance < 6) {
            if (conveyorTime.milliseconds() >= 1000) {
                full = true
            }
        } else {
            conveyorTime.reset()
            full = false
        }
        if (!this.forceReverse) {
            if (conveyorPower != 0.0 && !full) {
                if (!stop) {
                    stop = true
                }
                if (current <= 4 && backOnTime.milliseconds() >= 1000) {
                    if (!backOn) {
                        setConveyorMotorPower(conveyorPower)
                        backOn = true
                    }
                    timelock = backOnTime.milliseconds()
                    backOnAftertime.reset()
                } else {
                    if (backOn && backOnTime.milliseconds() >= timelock + 500) {
                        backOnTime.reset()
                        setConveyorMotorPower(-conveyorPower)
                        backOn = false
                    }
                }
            } else {
                if (stop) {
                    if (this.reverseBeforeStop && backOnAftertime.milliseconds() < 500) setConveyorMotorPower(-conveyorPower) else {
                        setConveyorMotorPower(0.0)
                        stop = false
                        backOn = false
                    }
                }
            }
        } else {
            setConveyorMotorPower(-1.0)
            backOn = false
            stop = true
        }
    }

    override fun updateOther() {
    }


    private fun setConveyorMotorPower(power: Double) {
        conveyorPowerSender.send(power)
    }

}