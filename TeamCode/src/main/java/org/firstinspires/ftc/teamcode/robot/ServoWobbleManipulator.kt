package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.teamcode.misc.CommandSender
import org.firstinspires.ftc.teamcode.superclasses.MultithreadRobotModule
import org.firstinspires.ftc.teamcode.superclasses.WobbleManipulator
import org.openftc.revextensions2.ExpansionHubServo

class ServoWobbleManipulator : MultithreadRobotModule(), WobbleManipulator {
    private lateinit var gripper: ExpansionHubServo
    private lateinit var leverArm: ExpansionHubServo

    @Config
    object WobbleServoPositions {
        @JvmField
        var gripperClose = 0.92
        @JvmField
        var gripperOpen = 0.19
        @JvmField
        var angleDown = 0.18
        @JvmField
        var angleMedium = 0.6
        @JvmField
        var angleUp = 1.0
    }

    private val closePositionSender = CommandSender { p: Double -> gripper.position = p }
    private val anglePositionSender = CommandSender { p: Double -> leverArm.position = p }
    private var isDown = false
    private var posAngle = WobbleManipulator.Position.UP
    private var leverArmPosition = 0.0
    private var gripperPosition = 0.0
    override fun initialize() {
        gripper = WoENHardware.gripper
        leverArm = WoENHardware.leverArm
        grabWobble(true)
        setAngle(WobbleManipulator.Position.UP)
        updateControlHub()
    }

    override fun grabWobble(dograb: Boolean) {
        gripperPosition =
            if (dograb) WobbleServoPositions.gripperClose else WobbleServoPositions.gripperOpen
    }

    override fun start() {

    }

    override fun updateControlHub() {
        closePositionSender.send(gripperPosition)
        anglePositionSender.send(leverArmPosition)
    }

    override fun updateExpansionHub() {
    }

    override fun updateOther() {
    }

    override fun upmediumdown(upmedium: Boolean, updown: Boolean) {
        if (upmedium && !updown) {
            setAngle(WobbleManipulator.Position.UP)
        } else if (updown && !upmedium) {
            if (!isDown) {
                isDown = true
                if (posAngle != WobbleManipulator.Position.MEDIUM) setAngle(WobbleManipulator.Position.MEDIUM) else setAngle(
                    WobbleManipulator.Position.DOWN
                )
            }
        } else isDown = false
    }

    override fun setAngle(Positions: WobbleManipulator.Position) {
        posAngle = Positions
        leverArmPosition = when (Positions) {
            WobbleManipulator.Position.UP -> WobbleServoPositions.angleUp
            WobbleManipulator.Position.DOWN -> WobbleServoPositions.angleDown
            WobbleManipulator.Position.MEDIUM -> WobbleServoPositions.angleMedium
        }
    }
}