package org.firstinspires.ftc.teamcode.robot.legacy

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.robot.WoENHardware
import org.firstinspires.ftc.teamcode.robot.WoENHardware.gripper
import org.firstinspires.ftc.teamcode.superclasses.MultithreadedRobotModule
import org.firstinspires.ftc.teamcode.superclasses.WobbleManipulator
import kotlin.math.abs

@Deprecated("")
class MotorWobbleManipulator : MultithreadedRobotModule(), WobbleManipulator {
    private val closeClose = 0.73
    private val closeOpen = 0.19
    private val minError = 15.0
    private val maxSpeed = 0.7
    private val kofP = 0.0015
    private val kofD = 0.00001
    private val leverTime = ElapsedTime()
    private lateinit var lever: DcMotorEx
    private lateinit var close: Servo
    private var ismed = false
    private var isdown = false
    private var isGrabbed = true
    private var posangle: Byte = 0
    private var pos = 0.0
    private var power = 0.0
    private var p = 0.0
    private var d = 0.0
    private var errorOld = 0.0
    private var error = 0.0
    private var oldpower = 0.0
    override fun initialize() {
        lever = WoENHardware.lever
        close = gripper
        lever.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        lever.mode = DcMotor.RunMode.RUN_USING_ENCODER
        lever.direction = DcMotorSimple.Direction.REVERSE
        lever.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        grabWobble(true)
    }

    override fun updateControlHub() {
    }

    override fun grabWobble(doGrab: Boolean) {
        if (doGrab != isGrabbed) {
            isGrabbed = doGrab
            if (doGrab) close.position = closeClose else close.position = closeOpen
        }
    }

    override fun updateExpansionHub() {
        error = pos - lever.currentPosition
        if (abs(error) > minError) {
            p = error * kofP
            d = (error - errorOld) * kofD
            power = p + d
            if (power > maxSpeed) power = maxSpeed
            if (power < -maxSpeed) power = -maxSpeed
            if (oldpower != power) {
                lever.power = power
                oldpower = power
            }
            errorOld = error
        } else {
            power = 0.0
            if (oldpower != power) {
                lever.power = 0.0
                oldpower = power
            }
        }
    }

    override fun updateOther() {
    }

    override fun upMediumDown(upMedium: Boolean, upDown: Boolean) {
        if (upMedium && !upDown) {
            if (!ismed) {
                ismed = true
                if (posangle.toInt() == 1) {
                    posangle = 0
                    setAngle(WobbleManipulator.Position.UP)
                } else {
                    posangle = 1
                    setAngle(WobbleManipulator.Position.MEDIUM)
                }
            }
        } else ismed = false
        if (upDown && !upMedium) {
            if (!isdown) {
                isdown = true
                if (posangle.toInt() == 2) {
                    posangle = 0
                    setAngle(WobbleManipulator.Position.UP)
                } else {
                    posangle = 2
                    setAngle(WobbleManipulator.Position.DOWN)
                }
            }
        } else isdown = false
    }

    override fun setAngle(Positions: WobbleManipulator.Position) {
        when (Positions) {
             WobbleManipulator.Position.UP -> setPosLever(0.0)
             WobbleManipulator.Position.DOWN -> setPosLever(920.0)
             WobbleManipulator.Position.MEDIUM -> setPosLever(550.0)
        }
    }

    fun setPosLever(Pos: Double) {
        pos = Pos
    }
}