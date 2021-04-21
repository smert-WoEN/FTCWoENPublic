package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.misc.CommandSender
import org.firstinspires.ftc.teamcode.robot.WoENrobot.shooter
import org.firstinspires.ftc.teamcode.superclasses.MultithreadedRobotModule
import org.firstinspires.ftc.teamcode.superclasses.LedStrip
import org.firstinspires.ftc.teamcode.superclasses.LedStrip.LedMode
import kotlin.math.PI
import kotlin.math.pow
import kotlin.math.sin

class LedStrip: MultithreadedRobotModule(), LedStrip {
    private lateinit var ledStrip1: DcMotorEx
    private lateinit var ledStrip2: DcMotorEx
    private val ledTime = ElapsedTime()
    private val ledOnOffTime = ElapsedTime()
    private val timeout = 1010.0
    private val setPowerLed1 = CommandSender({ p: Double -> ledStrip1.power = p })
    private val setPowerLed2 = CommandSender({ p: Double -> ledStrip2.power = p })


    override fun initialize() {
        ledStrip1 = WoENHardware.ledStrip1
        ledStrip2 = WoENHardware.ledStrip2
        ledStrip1.direction = DcMotorSimple.Direction.FORWARD
        ledStrip1.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        ledStrip1.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        ledStrip2.direction = DcMotorSimple.Direction.FORWARD
        ledStrip2.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        ledStrip2.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        ledTime.reset()
        ledOnOffTime.reset()
    }

    override fun updateControlHub() {

    }

    override fun updateExpansionHub() {
        if (ledOnOffTime.milliseconds() > timeout) {
            ledOnOffTime.reset()
            setLedMode(LedMode.INFORMSHOOTER)
        }
    }

    override fun updateOther() {

    }

    private fun smoothlyLedOn(led: CommandSender, time: Double = 1500.0) {
        val x = if (time != 0.0) 1 / time else 1.0
        when {
            ledTime.milliseconds() > time * 3 -> {
                ledTime.reset()
                led.send(0.0)
            }
            ledTime.milliseconds() < time -> {
                led.send(ledTime.milliseconds() * x)
            }
            ledTime.milliseconds() < time * 1.5 -> {
                led.send(1.0)
            }
            ledTime.milliseconds() < time * 2.5 -> {
                led.send(1.0 - (ledTime.milliseconds() - time * 1.5) * x)
            }
            else -> {
                led.send(0.0)
            }
        }
    }

    private fun smoothLed(led: CommandSender, time: Double = 1500.0, maxPower: Double = 1.0) {
        val x = if (time != 0.0) PI / time else 1.0
        led.send(sin(ledTime.milliseconds() * x).pow(2) * maxPower)
    }

    private fun informLed() {
        when {
            shooter.currentRpm == 0.0 -> {
                setPowerLed1.send(0.0)
                setPowerLed2.send(0.0)
            }
            shooter.isCorrectRpm(150.0) -> {
                setPowerLed1.send(1.0)
                setPowerLed2.send(0.0)
            }
            else -> {
                setPowerLed1.send(0.0)
                setPowerLed2.send(1.0)
            }
        }
    }

    private fun onLed(led: CommandSender, power: Double = 1.0) {
        led.send(power)
    }

    private fun offLed(led: CommandSender) {
        led.send(0.0)
    }

    fun setLedMode(mode: LedMode, ledPower1: Double = 1.0, ledPower2: Double = 1.0, ledTime1: Double = 1500.0, ledTime2: Double = 1500.0) {
        when (mode) {
             LedMode.ON -> {
                  onLed(setPowerLed1, ledPower1)
                  onLed(setPowerLed2, ledPower2)
             }
             LedMode.INFORMSHOOTER -> informLed()
             LedMode.SMOOTHLY -> {
                  smoothLed(setPowerLed1, ledTime1, ledPower1)
                  smoothLed(setPowerLed2, ledTime2, ledPower2)
             }
             LedMode.OFF -> {
                  offLed(setPowerLed1)
                  offLed(setPowerLed2)
             }
        }
    }
}

