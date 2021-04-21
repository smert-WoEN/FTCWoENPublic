@file:Suppress("ClassName")

package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.math.Pose2D
import org.firstinspires.ftc.teamcode.robot.WoENrobot.delay
import org.firstinspires.ftc.teamcode.robot.WoENrobot.movement
import kotlin.math.PI

@Autonomous
class Auto_Sandbox : AutoOpMode() {
    override fun main() {
        while (opModeIsActive()) {
            //movement.pos(Pose2D(startPosition.x, startPosition.y+200.0, 0.0))
            movement.pos(Pose2D(startPosition.x, startPosition.y+20.0, PI/2))
            movement.pos(Pose2D(startPosition.x, startPosition.y+20.0, 0.0))
            // shootPOWERSHOTAngle()
        }
    }
}