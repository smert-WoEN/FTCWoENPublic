@file:Suppress("ClassName")

package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.math.Pose2D
import org.firstinspires.ftc.teamcode.robot.WoENrobot.delay
import org.firstinspires.ftc.teamcode.robot.WoENrobot.movement

@Autonomous
class Auto_Sandbox : AutoOpMode() {
    override fun main() {
        movement.pos(Pose2D(50.0, -100.0, Math.toRadians(0.0)))
        movement.pos(Pose2D(50.0, 100.0, Math.toRadians(180.0)))
        movement.pos(Pose2D(50.0, -100.0, Math.toRadians(180.0)))
        movement.pos(Pose2D(50.0, -100.0, Math.toRadians(0.0)))
        movement.pos(startPosition)
        while (opModeIsActive()) {
            delay(1.0)
        }
    }
}