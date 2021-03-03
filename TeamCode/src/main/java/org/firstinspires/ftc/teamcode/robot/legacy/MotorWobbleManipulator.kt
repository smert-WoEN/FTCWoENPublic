package org.firstinspires.ftc.teamcode.robot.legacy

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.robot.WoENHardware
import org.firstinspires.ftc.teamcode.robot.WoENHardware.gripper
import org.firstinspires.ftc.teamcode.superclasses.MultithreadRobotModule
import org.firstinspires.ftc.teamcode.superclasses.WobbleManipulator
import kotlin.math.abs

@Deprecated("")
class MotorWobbleManipulator : MultithreadRobotModule(), WobbleManipulator {
    private val closeClose = 0.73
    private val closeOpen = 0.19
    private val minerror = 15.0
    private val maxspeed = 0.7
    private val kofP = 0.0015
    private val kofd = 0.00001
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

    override fun grabWobble(dograb: Boolean) {
        if (dograb != isGrabbed) {
            isGrabbed = dograb
            if (dograb) close.position = closeClose else close.position = closeOpen
        }
    }

    override fun updateExpansionHub() {
        error = pos - lever.currentPosition
        if (abs(error) > minerror) {
            p = error * kofP
            d = (error - errorOld) * kofd
            power = p + d
            if (power > maxspeed) power = maxspeed
            if (power < -maxspeed) power = -maxspeed
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

    override fun upmediumdown(upmedium: Boolean, updown: Boolean) {
        if (upmedium && !updown) {
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
        if (updown && !upmedium) {
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
            WobbleManipulator.Position.UP -> setposlever(0.0)
            WobbleManipulator.Position.DOWN -> setposlever(920.0)
            WobbleManipulator.Position.MEDIUM -> setposlever(550.0)
        }
    }

    fun setposlever(Pos: Double) {
        pos = Pos
    }
}