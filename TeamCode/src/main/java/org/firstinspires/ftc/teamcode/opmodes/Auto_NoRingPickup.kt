@file:Suppress("ClassName")

package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.avoidRingStack
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.moveWobble
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.park
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.shootHighGoal
import org.firstinspires.ftc.teamcode.opmodes.MovementMacros.shooting
import org.firstinspires.ftc.teamcode.robot.WoENrobot.delay

@Autonomous
class Auto_NoRingPickup : AutoOpMode() {
    override fun main() {
        avoidRingStack()
        shootHighGoal()
        delay(0.0)
        moveWobble()
        park()
    }
}