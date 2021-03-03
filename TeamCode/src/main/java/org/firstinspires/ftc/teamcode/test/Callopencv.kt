package org.firstinspires.ftc.teamcode.test

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.robot.WoENrobot

@TeleOp
@Disabled
class Callopencv : LinearOpMode() {
    override fun runOpMode() {
        // WoENrobot.forceInitRobot(this);
        WoENrobot.openCVNode.initialize(this)
        waitForStart()
        FtcDashboard.getInstance().startCameraStream(WoENrobot.openCVNode.webcam, 0.0)
        while (opModeIsActive()) {
            telemetry.addData("mean", WoENrobot.openCVNode.mean)
            telemetry.addData("getAspectRatio", WoENrobot.openCVNode.aspectRatio)
            telemetry.addData("getStackSize", WoENrobot.openCVNode.stackSize)
            telemetry.update()
        }
        FtcDashboard.getInstance().stopCameraStream()
    }
}