package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.math.Pose2D
import org.firstinspires.ftc.teamcode.misc.HSVRGB
import org.firstinspires.ftc.teamcode.misc.SinglePressButton
import org.firstinspires.ftc.teamcode.robot.WoENrobot.fullInitWithCV
import org.firstinspires.ftc.teamcode.robot.WoENrobot.movement
import org.firstinspires.ftc.teamcode.robot.WoENrobot.odometry
import org.firstinspires.ftc.teamcode.robot.WoENrobot.openCVNode
import org.firstinspires.ftc.teamcode.robot.WoENrobot.runTime
import org.firstinspires.ftc.teamcode.robot.WoENrobot.setLedColors
import org.firstinspires.ftc.teamcode.robot.WoENrobot.startRobot

open class AutoOpMode : LinearOpMode() {
    private var xSign: Int = 1
    private var sideSign: Int = 1
    private var thereAreTwoGamepads = false
    private var delayAtStart = 0.0
    open fun main() {}
    val startPosition: Pose2D
        get() = Pose2D(93.91741046 * xSign + 30.1416 * sideSign, -156.12089687, 0.0)

    override fun waitForStart() {
        while (!isStarted) {
            startLoop()
        }
        super.waitForStart()
    }

    private var delayAtStartIncrementPresser = SinglePressButton { gamepad1.dpad_up }
    private var delayAtStartDecrementPresser = SinglePressButton { gamepad1.dpad_down }
    private fun startLoop() {
        val color = HSVRGB.convert((runTime.seconds() * 50).toFloat() % 360, 100f, 50f)
        setLedColors(color.x.toInt(), color.y.toInt(), color.z.toInt())
        val indicator = if (runTime.seconds() % 1 > 0.5) if (runTime.seconds() % 1 > 0.75) "/" else "|" else if (runTime.seconds() % 1 > 0.25) "—" else "\\"
        telemetry.addLine(
            "Use gamepad 1 X/B to select alliance color, dpad L/R to select alliance side, dpad UP/DOWN to change starting delay. $indicator")
        xSign = if (gamepad1.b) 1 else if (gamepad1.x) -1 else xSign
        sideSign = if (gamepad1.dpad_right || gamepad1.left_stick_x > 0.5) 1 else if (gamepad1.dpad_left || gamepad1.left_stick_x < -0.5) -1 else sideSign
        delayAtStart = Range.clip(
            delayAtStart + (if (delayAtStartIncrementPresser.get()) 500 else 0) - if (delayAtStartDecrementPresser.get()) 500 else 0,
            0.0, 30000.0)
        telemetry.addData("Alliance", if (xSign == 1) "RED" else "BLUE")
        telemetry.addData("Tape Side", if (sideSign == 1) "RIGHT" else "LEFT")
        telemetry.addData("Starting delay [ms]", delayAtStart)
        telemetry.addLine("")
        thereAreTwoGamepads = gamepad2.start || gamepad2.b || thereAreTwoGamepads
        telemetry.addData("OpenCV Stack size", openCVNode.stackSize)
        //dashboardPacket.put("OpenCV Stack size", openCVNode.stackSize)
        if (thereAreTwoGamepads) telemetry.addLine("Second gamepad detected")
        telemetry.update()
    }

    override fun runOpMode() {
        fullInitWithCV(this)
        startRobot()
        movement.setActiveBraking(true)
        if (isStopRequested) return
        openCVNode.stopCam()
        MovementMacros.setSettings(xSign, sideSign)
        odometry.robotCoordinates = startPosition
        try {
            main()
        } finally {
            setLedColors(0, 128, 128)
            telemetry.addData("Status", "Program finished ($runtime)")
            telemetry.update()
        }
    }
}